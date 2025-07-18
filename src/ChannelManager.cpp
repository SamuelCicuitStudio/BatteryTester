#include "ChannelManager.h"
#include "Config.h"


/**
 * @brief Constructs a ChannelManager instance for a specific charging channel.
 *
 * Initializes internal references to the I2C multiplexer channel, associated GPIO manager,
 * shift register pin assignments for RGB LED control, OTG, CE, and load control.
 * Also sets the ADC and digital input pins used for real-time monitoring.
 *
 * On construction:
 * - Sets the TCA9548A I2C mux channel bit for future communication.
 * - Disables all load outputs (LD01–LD04) globally via the GpioManager to ensure safe startup.
 * - Sets the RGB LED color to red (🔴), indicating the charger is initially disabled.
 *
 * @param muxChannel The channel number (0–7) on the TCA9548A I2C multiplexer to activate for this device.
 * @param mux Reference to the shared Tca9548aMux used to switch active I2C channels.
 * @param gpio Pointer to the shared GpioManager used for shift register control.
 * @param rPin Shift register pin controlling the red LED for this channel.
 * @param gPin Shift register pin controlling the green LED for this channel.
 * @param bPin Shift register pin controlling the blue LED for this channel.
 * @param otgPin Shift register pin used to enable or disable OTG boost mode.
 * @param cePin Shift register pin used to control the CE (charger enable) line.
 * @param loadPin Shift register pin used to activate the discharge load.
 * @param voltageAdcPin ADC pin connected to the current sensor for this channel.
 * @param intPin GPIO pin connected to the BQ2589X INT output.
 * @param statPin GPIO pin connected to the BQ2589X STAT output.
 * @param pgPin GPIO pin connected to the BQ2589X PG (Power Good) output.
 */
ChannelManager::ChannelManager(uint8_t muxChannel,
                               Tca9548aMux* mux,
                               GpioManager* gpio,
                               ShiftPin rPin, ShiftPin gPin, ShiftPin bPin,
                               ShiftPin otgPin, ShiftPin cePin,
                               ShiftPin loadPin, uint8_t voltageAdcPin,
                               uint8_t intPin, uint8_t statPin, uint8_t pgPin)
    : mux(mux), muxChannel(muxChannel),
      gpioManager(gpio),
      rPin(rPin), gPin(gPin), bPin(bPin),
      cePin(cePin), otgPin(otgPin),
      loadPin(loadPin), voltageAdcPin(voltageAdcPin),
      intPin(intPin), statPin(statPin), pgPin(pgPin)
{
    DEBUG_PRINTLN("-----------------------------------------------------------");
    DEBUG_PRINTF("🔌 Initializing ChannelManager for channel %d\n", muxChannel);
    DEBUG_PRINTLN("-----------------------------------------------------------");

    // Display pin configuration
    DEBUG_PRINTLN("→ Shift register output pin mapping:");
    DEBUG_PRINTF("   - RGB Pins: R=%d, G=%d, B=%d\n", rPin, gPin, bPin);
    DEBUG_PRINTF("   - CE Pin: %d | OTG Pin: %d | LOAD Pin: %d\n", cePin, otgPin, loadPin);
    DEBUG_PRINTF("→ Voltage ADC Pin: GPIO %d\n", voltageAdcPin);
    DEBUG_PRINTF("→ Status Pins: INT=%d | STAT=%d | PG=%d\n", intPin, statPin, pgPin);

    // Disable all load pins globally on startup (LOW = inactive)
    gpioManager->setShiftPin(SHIFT_LD01, LOW);
    gpioManager->setShiftPin(SHIFT_LD02, LOW);
    gpioManager->setShiftPin(SHIFT_LD03, LOW);
    gpioManager->setShiftPin(SHIFT_LD04, LOW);
    gpioManager->applyShiftState();
    DEBUG_PRINTLN("✓ All LOAD control pins set to HIGH (disabled)");

    // Set LED to RED to indicate charger is currently OFF
    setRgbColor(true, false, false);
    DEBUG_PRINTLN("✓ RGB LED set to RED (charger OFF)");
}




/**
 * @brief Initializes the BQ2589X charger driver and activates the I2C mux channel.
 *
 * This function must be called during system startup to configure the BQ2589X charger.
 * It performs the following actions:
 * - Stores a reference to the I2C bus used for communication.
 * - Activates the corresponding TCA9548A multiplexer channel for this charging port.
 * - Initializes the BQ2589X driver using the provided I2C bus and default address.
 *
 * @param wire Reference to the shared TwoWire (I2C) bus used for communication.
 */
void ChannelManager::begin() {
    DEBUG_PRINTF("🔁 Initializing Channel %d via TCA9548A multiplexer...\n", muxChannel);

    // Activate the channel using the TCA9548A multiplexer
    if (!mux->selectChannel(muxChannel)) {
        DEBUG_PRINTF("❌ Failed to select mux channel %d\n", muxChannel);
        return;
    }

    DEBUG_PRINTF("✓ Mux channel %d activated via TCA9548A (Addr: 0x%02X)\n", muxChannel, TCA9548A_ADDR);

    // Initialize BQ2589x charger over the currently selected I2C channel
    bq.begin(mux->_wire, BQ2589X_I2C_ADDR);  // Access _wire pointer from TCA if exposed publicly
    DEBUG_PRINTLN("✓ BQ2589x charger initialized via I2C.");
}


/**
 * @brief Returns a reference to the internal BQ2589X charger driver instance.
 *
 * This allows external code to directly access low-level functions or registers
 * of the BQ2589X charger chip associated with this specific channel.
 *
 * @return Reference to the internal `bq2589x` object.
 */
bq2589x& ChannelManager::getBQ() {
    return bq;
}
/**
 * @brief Returns the I2C address of the TCA9548A multiplexer.
 *
 * This address is used to communicate with the I2C multiplexer that controls
 * access to the individual battery charger channels.
 *
 * @return 8-bit I2C address of the TCA9548A multiplexer.
 */
uint8_t ChannelManager::getMuxAddress() const {
    return mux ? mux->getAddress() : 0xFF;  // 0xFF as fallback if null
}

/**
 * @brief Returns the bitmask representing the active I2C mux channel.
 *
 * This bitmask is used to select the corresponding channel on the TCA9548A
 * I2C multiplexer. The value is typically (1 << muxChannel).
 *
 * @return 8-bit value representing the mux channel bitmask.
 */
uint8_t ChannelManager::getMuxChannel() const {
    return muxChannel;
}

/**
 * @brief Sets the RGB LED color for the channel.
 *
 * This function controls the red, green, and blue components of the RGB LED
 * associated with the charging channel by setting the corresponding shift register
 * outputs. Each color is enabled or disabled based on the provided boolean flags.
 *
 * @param r True to turn on the red component, false to turn it off.
 * @param g True to turn on the green component, false to turn it off.
 * @param b True to turn on the blue component, false to turn it off.
 */
void ChannelManager::setRgbColor(bool r, bool g, bool b) {
    // Set RGB LED color using shift register pins
    gpioManager->setShiftPin(rPin, r);  // Red component
    gpioManager->setShiftPin(gPin, g);  // Green component
    gpioManager->setShiftPin(bPin, b);  // Blue component

    gpioManager->applyShiftState();     // Apply the updated state to shift register

    DEBUG_PRINTF("🔴 RGB Color Set - R:%d G:%d B:%d for Channel %d\n", r, g, b, muxChannel);
}


/**
 * @brief Controls the CE (Charge Enable) pin for the charger.
 *
 * This function enables or disables the charger hardware by setting the CE pin state
 * through the shift register. The CE pin is typically active-low, so enabling the charger
 * usually means setting this pin to LOW.
 *
 * @param enabled If true, the CE pin is set active (typically LOW). If false, the CE pin is deactivated (typically HIGH).
 */
void ChannelManager::setCE(bool enabled) {
    gpioManager->setShiftPin(cePin, enabled);
    gpioManager->applyShiftState();
}

/**
 * @brief Enables or disables OTG (On-The-Go) mode on the charger.
 *
 * This function sets the OTG control pin via the shift register. OTG mode allows the charger
 * to act as a power source (boost converter), typically supplying 5V to downstream devices.
 * 
 * @param enabled If true, OTG mode is activated. If false, OTG mode is disabled.
 */
void ChannelManager::setOTG(bool enabled) {
    gpioManager->setShiftPin(otgPin, enabled);
    gpioManager->applyShiftState();
}

/**
 * @brief Reads the INT (Interrupt) pin state.
 *
 * The INT pin is active-low and indicates that an interrupt has occurred
 * on the charger (e.g., charge complete, fault condition, etc.).
 *
 * @return true if the INT pin is LOW (active), false if HIGH (inactive).
 */
bool ChannelManager::readInt() {
    return digitalRead(intPin) == LOW;
}

/**
 * @brief Reads the STAT (Status) pin state.
 *
 * The STAT pin reflects charging status:
 * - LOW: Charging in progress
 * - HIGH: Not charging / Charge complete
 * 
 * @return true if STAT pin is HIGH, false if LOW.
 */
bool ChannelManager::readStat() {
    return digitalRead(statPin);
}

/**
 * @brief Reads the PG (Power Good) pin state.
 *
 * The PG pin indicates whether a valid power source is connected to VBUS.
 * - HIGH: VBUS is valid and present
 * - LOW: No valid VBUS detected
 *
 * @return true if PG pin is HIGH (power good), false otherwise.
 */
bool ChannelManager::readPGood() {
    return digitalRead(pgPin);
}

/**
 * @brief Initializes the BQ2589x charger on the current I2C mux channel.
 *
 * This function performs all necessary setup to prepare the BQ2589x charger for use:
 * - Activates the correct TCA9548A I2C mux channel.
 * - Initializes communication with the charger IC.
 * - Disables the watchdog timer to prevent unintended resets.
 * - Starts continuous ADC mode (for live telemetry).
 * - Ensures the charger is disabled for a safe start.
 * - Applies default charging voltage and current settings from configuration.
 * - Configures OTG (boost) mode parameters.
 * - Disables both CE (charge enable) and OTG outputs by default.
 * - Updates the RGB LED to reflect the initial charger state.
 *
 * This should be called once after boot or reset to initialize the channel safely.
 */
void ChannelManager::initCharger() {
    DEBUG_PRINTF("🔌 Initializing Charger on Channel %d\n", muxChannel);

    // Select the appropriate I2C multiplexer channel for this charger
    if (!mux->selectChannel(muxChannel)) {
        DEBUG_PRINTF("❌ Failed to select mux channel %d\n", muxChannel);
        return;
    }

    // Initialize BQ2589x charger driver on the I2C bus
    bq.begin(mux->_wire, BQ2589X_I2C_ADDR);

    // Configure charger with default behavior
    bq.disable_watchdog_timer();                        // Prevent unintentional resets
    bq.adc_start(false);                                // Start ADC in manual mode
    bq.disable_charger();                               // Disable charging initially
    bq.set_charge_voltage(DEFAULT_CHARGE_VOLTAGE_MV);   // Default: 4200 mV
    bq.set_charge_current(DEFAULT_CHARGE_CURRENT_MA);   // Default: 1500 mA
    bq.set_otg_voltage(DEFAULT_BOOST_VOLTAGE_MV);       // Default: 5000 mV
    bq.set_otg_current(DEFAULT_BOOST_CURRENT_MA);       // Default: 1000 mA

    // Ensure CE and OTG outputs are disabled
    setCE(false);
    setOTG(false);

    // Update LED color to reflect current charger status
    updateLedFromStatus();

    DEBUG_PRINTLN("✅ Charger Initialization Complete");
}



/**
 * @brief Sets the charging voltage for the BQ2589x charger.
 *
 * This function selects the active I2C mux channel, then sets the target 
 * charging voltage (in millivolts) for the associated battery charger.
 *
 * @param voltage_mV Charging voltage in millivolts (e.g., 4200 for 4.2V).
 *
 * @note This value should be within the supported range of the BQ2589x IC.
 */
void ChannelManager::setChargingVoltage(uint16_t voltage_mV) {
    if (!mux->selectChannel(muxChannel)) {
        DEBUG_PRINTF("❌ Failed to select mux channel %d\n", muxChannel);
        return;
    }
    bq.set_charge_voltage(voltage_mV);
}


/**
 * @brief Sets the charging current for the BQ2589x charger.
 *
 * This function selects the active I2C mux channel, then configures the 
 * target charge current (in milliamps) to be delivered to the battery.
 *
 * @param current_mA Charging current in milliamps (e.g., 1500 for 1.5A).
 *
 * @note This value should not exceed the safe current for the battery or 
 * the limits of the BQ2589x IC.
 */
void ChannelManager::setChargingCurrent(uint16_t current_mA) {
    if (!mux->selectChannel(muxChannel)) {
        DEBUG_PRINTF("❌ Failed to select mux channel %d\n", muxChannel);
        return;
    }
    bq.set_charge_current(current_mA);
}

/**
 * @brief Measures an approximate battery capacity value in mWh.
 *
 * This function estimates the battery capacity by briefly activating a discharge load
 * and observing the voltage and current difference. Battery voltage is measured using 
 * the BQ2589x ADC over I²C, while current is derived from an analog current sensor.
 *
 * Steps:
 * - Selects the correct TCA9548A multiplexer channel for this battery.
 * - Enables a known load (e.g., 2Ω resistor) via shift register output.
 * - Measures battery voltage before and after load activation using the BQ2589x ADC.
 * - Measures current before and after using analogRead on a current sensor output.
 * - Calculates ΔI (current delta) and computes instantaneous discharge power.
 * - Approximates energy released during ~100ms discharge and converts it to mWh.
 *
 * @return Estimated discharged energy in milliWatt-hours (mWh), useful for tracking
 *         battery response and degradation over time.
 *
 * @note This is a rapid, relative measurement. Accuracy depends on:
 *       - Load resistor value and tolerance
 *       - Analog current sensor linearity and offset
 *       - BQ2589x voltage ADC precision
 *       - Timing accuracy and electrical noise
 */
float ChannelManager::measureCapacity() {
    // === Step 0: Select correct TCA9548A channel
    if (!mux->selectChannel(muxChannel)) {
        DEBUG_PRINTF("❌ Failed to select mux channel %d\n", muxChannel);
        return 0.0f;
    }

    // === Step 1: Activate load (pull current from battery)
    gpioManager->setShiftPin(loadPin, HIGH);  // Enable 2Ω load
    gpioManager->applyShiftState();
    delay(50);  // Let current stabilize

    // === Step 2: Read voltage before load removal from BQ chip (mV)
    int voltageBefore_mV = bq.adc_read_battery_volt();
    float voltageBefore = voltageBefore_mV / 1000.0f;

    // === Step 3: Read current from analog sensor
    uint16_t adcRawBefore = analogRead(voltageAdcPin);
    float sensorVoltageBefore = (adcRawBefore * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
    float currentBefore = (sensorVoltageBefore - CURRENT_SENSOR_ZERO_VOLTAGE) / CURRENT_SENSOR_SENSITIVITY;

    // === Step 4: Disable load
    gpioManager->setShiftPin(loadPin, LOW);
    gpioManager->applyShiftState();
    delay(100);

    // === Step 5: Read voltage again after load is removed
    int voltageAfter_mV = bq.adc_read_battery_volt();
    float voltageAfter = voltageAfter_mV / 1000.0f;

    // === Step 6: Read current again after load is off
    uint16_t adcRawAfter = analogRead(voltageAdcPin);
    float sensorVoltageAfter = (adcRawAfter * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
    float currentAfter = (sensorVoltageAfter - CURRENT_SENSOR_ZERO_VOLTAGE) / CURRENT_SENSOR_SENSITIVITY;

    // === Step 7: Re-disable load just in case
    gpioManager->setShiftPin(loadPin, HIGH);
    gpioManager->applyShiftState();

    // === Step 8: Compute delta current and power
    float deltaCurrent = currentBefore - currentAfter;
    float power = deltaCurrent * deltaCurrent * LOAD_RESISTOR_OHMS;  // P = I²R
    float energy_mWs = power * 100.0f;  // 100ms discharge window

    // === Step 9: Convert to mWh
    float capacity_mWh = energy_mWs / 3600.0f;

    DEBUG_PRINTF("⚡ Channel %d:\n", muxChannel);
    DEBUG_PRINTF("   Vbefore=%.3fV | Vafter=%.3fV\n", voltageBefore, voltageAfter);
    DEBUG_PRINTF("   Ibefore=%.3fmA | Iafter=%.3fmA | ΔI=%.3fmA\n", currentBefore * 1000.0f, currentAfter * 1000.0f, deltaCurrent * 1000.0f);
    DEBUG_PRINTF("   Power=%.2fmW | Capacity=%.4fmWh\n", power, capacity_mWh);

    return capacity_mWh;
}


/**
 * @brief Sends a JSON-formatted capacity report over the Serial interface.
 *
 * This function collects and transmits essential battery diagnostics 
 * in real-time, including:
 * - Mux channel number
 * - Battery voltage (from ADC)
 * - Estimated capacity in mWh (via load test)
 * - Charging status (true/false)
 *
 * The output format is:
 * {
 *   "channel": <muxChannel>,
 *   "voltage": <voltage in V>,
 *   "capacity_mWh": <estimated capacity>,
 *   "charging": <true|false>
 * }
 *
 * Example output:
 * `{"channel":1,"voltage":3.82,"capacity_mWh":15.43,"charging":true}`
 *
 * @note This function is useful for logging, debugging, or displaying 
 * battery state in a UI. It assumes Serial is already initialized.
 */
void ChannelManager::sendCapacityReport() {
    // Select the correct TCA9548A mux channel
    mux->selectChannel(muxChannel);

    // Measure voltage using BQ2589x ADC
    int voltage_mV = bq.adc_read_battery_volt();
    float voltage = voltage_mV / 1000.0f;

    // Estimate capacity in mWh
    float capacity = measureCapacity();

    // Send JSON-formatted report over Serial
    Serial.print(F("{\"channel\":"));
    Serial.print(muxChannel);
    Serial.print(F(",\"voltage\":"));
    Serial.print(voltage, 2);
    Serial.print(F(",\"capacity_mWh\":"));
    Serial.print(capacity, 2);
    Serial.print(F(",\"charging\":"));
    Serial.print(isCharging() ? F("true") : F("false"));
    Serial.println(F("}"));
}

/**
 * @brief Starts the charging process for the current battery channel.
 *
 * This function:
 * - Activates the corresponding I2C multiplexer channel.
 * - Enables the BQ2589x charger via I2C.
 * - Sets the CE (Charge Enable) pin to active state using the shift register.
 * - Updates the RGB LED to indicate the charging status (🟢 Green).
 *
 * It is typically called after charger initialization or when re-enabling charging manually.
 *
 * @note Charging parameters (voltage, current) must be configured before calling this function.
 */
void ChannelManager::startCharging() {
    // Select the appropriate TCA9548A mux channel
    mux->selectChannel(muxChannel);

    // Enable charging on the BQ2589x
    bq.enable_charger();

    // Enable CE pin via shift register
    setCE(true);

    // Update LED to reflect charging status
    updateLedFromStatus();
}

/**
 * @brief Stops the charging process for the current battery channel.
 *
 * This function:
 * - Selects the correct I2C multiplexer channel.
 * - Disables the BQ2589x charger over I2C.
 * - Deactivates the CE (Charge Enable) pin using the shift register.
 * - Updates the RGB LED to reflect the new status (🔴 Red for charger off).
 *
 * It is typically used when charging needs to be halted due to system logic,
 * user command, safety conditions, or full charge completion.
 */
void ChannelManager::stopCharging() {
    // Select the appropriate TCA9548A mux channel
    mux->selectChannel(muxChannel);
    // Disable the charger
    bq.disable_charger();
    // Disable CE pin via shift register
    setCE(false);
    // Update RGB LED to reflect charger off status
    updateLedFromStatus();
}

/**
 * @brief Checks whether the charger is currently active for the selected channel.
 *
 * This function:
 * - Selects the appropriate I2C mux channel to access the correct BQ2589x device.
 * - Queries the charger via I2C to determine if charging is enabled.
 *
 * @return `true` if the charger is currently enabled and charging, `false` otherwise.
 */
bool ChannelManager::isCharging() {
    // Select the correct channel on the I2C mux
    mux->selectChannel(muxChannel);
    // Query the BQ2589x charger for charging state
    return bq.is_charge_enabled();
}

/**
 * @brief Retrieves the real-time charging and system status for the current channel.
 *
 * This function:
 * - Selects the active I2C mux channel to communicate with the correct BQ2589x charger.
 * - Reads multiple analog and digital metrics from the charger, including:
 *   - Battery voltage
 *   - System voltage (VSYS)
 *   - VBUS input voltage
 *   - Charge current
 *   - Temperature (from internal sensor)
 *   - Configured charge voltage and current
 *   - Charger enable state
 *   - Charging completion status
 *   - Human-readable charging status string
 * - Reads GPIO-based hardware states:
 *   - Power Good (PG) pin
 *   - STAT pin
 *   - INT (interrupt) pin
 *
 * @return A fully populated `ChargingStatus` struct containing all relevant live data.
 */
ChargingStatus ChannelManager::getStatus() {
    // Select the correct TCA9548A mux channel
    mux->selectChannel(muxChannel);

    ChargingStatus status;
    status.channel = muxChannel;
    status.batteryVoltage = bq.adc_read_battery_volt() / 1000.0f;
    status.systemVoltage = bq.adc_read_sys_volt() / 1000.0f;
    status.vbusVoltage = bq.adc_read_vbus_volt() / 1000.0f;
    status.chargeCurrent = bq.adc_read_charge_current();
    status.temperature = bq.adc_read_temperature() / 10.0f;
    status.chargeVoltage = bq.get_charge_voltage();
    status.chargeCurrentSet = bq.get_charge_current();
    status.chargerEnabled = bq.is_charge_enabled();
    status.chargingDone = bq.is_charge_done();
    status.chargingStatusText = bq.get_charging_status_text();
    status.pg = readPGood();
    status.stat = readStat();
    status.intLow = readInt();

    DEBUG_PRINTLN("======= Channel Status Report =======");
    DEBUG_PRINTF("Channel            : %d\n", status.channel);
    DEBUG_PRINTF("Battery Voltage    : %.3f V\n", status.batteryVoltage);
    DEBUG_PRINTF("System Voltage     : %.3f V\n", status.systemVoltage);
    DEBUG_PRINTF("VBUS Voltage       : %.3f V\n", status.vbusVoltage);
    DEBUG_PRINTF("Charge Current     : %d mA\n", status.chargeCurrent);
    DEBUG_PRINTF("Temperature        : %.1f °C\n", status.temperature);
    DEBUG_PRINTF("Charge Voltage Set : %d mV\n", status.chargeVoltage);
    DEBUG_PRINTF("Charge Current Set : %d mA\n", status.chargeCurrentSet);
    DEBUG_PRINTF("Charger Enabled    : %s\n", status.chargerEnabled ? "Yes" : "No");
    DEBUG_PRINTF("Charging Done      : %s\n", status.chargingDone ? "Yes" : "No");
    DEBUG_PRINTF("Status Text        : %s\n", status.chargingStatusText.c_str());
    DEBUG_PRINTF("PG (Power Good)    : %s\n", status.pg ? "HIGH" : "LOW");
    DEBUG_PRINTF("STAT Pin           : %s\n", status.stat ? "HIGH" : "LOW");
    DEBUG_PRINTF("INT Pin (Active?)  : %s\n", status.intLow ? "LOW (Active)" : "HIGH (Idle)");
    DEBUG_PRINTLN("=====================================");

    return status;
}

/**
 * @brief Updates the RGB LED color to reflect the current charging state.
 *
 * This function uses the current charging status to visually indicate:
 * - 🔴 Red: Charger is disabled (`chargerEnabled == false`)
 * - 🔵 Blue: Charging is complete (`chargingDone == true`)
 * - 🟢 Green: Charging is actively in progress
 *
 * It reads the current status using `getStatus()` and then sets the appropriate
 * RGB pins using `setRgbColor()`, which updates the shift register.
 *
 * This provides immediate visual feedback per channel for system monitoring.
 */
void ChannelManager::updateLedFromStatus() {
    ChargingStatus status = getStatus();

    if (!status.chargerEnabled) {
        setRgbColor(true, false, false);   // 🔴 Red
    } else if (status.chargingDone) {
        setRgbColor(false, false, true);   // 🔵 Blue
    } else {
        setRgbColor(false, true, false);   // 🟢 Green
    }
}
