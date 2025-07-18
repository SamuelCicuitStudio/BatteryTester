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
                               GpioManager* gpio,
                               ShiftPin rPin, ShiftPin gPin, ShiftPin bPin,
                               ShiftPin otgPin, ShiftPin cePin,
                               ShiftPin loadPin, uint8_t voltageAdcPin,
                               uint8_t intPin, uint8_t statPin, uint8_t pgPin)
    : gpioManager(gpio),
      rPin(rPin), gPin(gPin), bPin(bPin),
      cePin(cePin), otgPin(otgPin),
      loadPin(loadPin), voltageAdcPin(voltageAdcPin),
      intPin(intPin), statPin(statPin), pgPin(pgPin)
{
    DEBUG_PRINTLN("-----------------------------------------------------------");
    DEBUG_PRINTF("🔌 Initializing ChannelManager for channel %d\n", muxChannel);
    DEBUG_PRINTLN("-----------------------------------------------------------");

    tcaAddress = TCA9548A_ADDR;
    this->muxChannel = muxChannel;
    muxChannelBit = (1 << (muxChannel & 0x07));
    DEBUG_PRINTF("I2C Multiplexer Address: 0x%02X | Channel Bit: 0x%02X\n", tcaAddress, muxChannelBit);

    // Display pin configuration
    DEBUG_PRINTLN("→ Shift register output pin mapping:");
    DEBUG_PRINTF("   - RGB Pins: R=%d, G=%d, B=%d\n", rPin, gPin, bPin);
    DEBUG_PRINTF("   - CE Pin: %d | OTG Pin: %d | LOAD Pin: %d\n", cePin, otgPin, loadPin);
    DEBUG_PRINTF("→ Voltage ADC Pin: GPIO %d\n", voltageAdcPin);
    DEBUG_PRINTF("→ Status Pins: INT=%d | STAT=%d | PG=%d\n", intPin, statPin, pgPin);

    // Disable all load pins globally on startup (HIGH = inactive)
    gpioManager->setShiftPin(SHIFT_LD01, HIGH);
    gpioManager->setShiftPin(SHIFT_LD02, HIGH);
    gpioManager->setShiftPin(SHIFT_LD03, HIGH);
    gpioManager->setShiftPin(SHIFT_LD04, HIGH);
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
void ChannelManager::begin(TwoWire& wire) {
    DEBUG_PRINTF("🔁 Initializing Channel %d I2C interface...\n", muxChannel);

    _wire = &wire;

    // Activate the channel on the TCA9548A multiplexer
    setMuxActive(*_wire);
    DEBUG_PRINTF("✓ Mux channel 0x%02X activated on TCA9548A (Addr: 0x%02X)\n", muxChannelBit, tcaAddress);

    // Initialize BQ2589x charger over I2C
    bq.begin(_wire, BQ2589X_I2C_ADDR);
    DEBUG_PRINTLN("✓ BQ2589x charger initialized via I2C.");
}

/**
 * @brief Activates the I2C multiplexer (TCA9548A) channel assigned to this charging channel.
 *
 * Sends a command to the TCA9548A to enable only the specific mux channel associated
 * with this `ChannelManager` instance. This ensures that I2C communication with the
 * BQ2589X charger chip is routed correctly.
 *
 * This function is called internally before any I2C transaction to ensure the correct
 * device is selected via the TCA9548A.
 *
 * @param wire Reference to the TwoWire (I2C) bus used to communicate with the multiplexer.
 */
void ChannelManager::setMuxActive(TwoWire& wire) {
    wire.beginTransmission(tcaAddress);
    wire.write(muxChannelBit);
    wire.endTransmission();
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
    return tcaAddress;
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
    return muxChannelBit;
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
    setMuxActive(*_wire);
    // Initialize BQ2589x charger driver on the I2C bus
    bq.begin(_wire, BQ2589X_I2C_ADDR);
    // Disable the watchdog timer to avoid unintended resets
    bq.disable_watchdog_timer();
    // Start ADC in manual mode (not continuous)
    bq.adc_start(false);
    // Ensure charger is disabled on startup
    bq.disable_charger();
    // Set default charge parameters
    bq.set_charge_voltage(DEFAULT_CHARGE_VOLTAGE_MV);  // e.g. 4200 mV
    bq.set_charge_current(DEFAULT_CHARGE_CURRENT_MA);  // e.g. 1500 mA
    bq.set_otg_voltage(DEFAULT_BOOST_VOLTAGE_MV);      // e.g. 5000 mV
    bq.set_otg_current(DEFAULT_BOOST_CURRENT_MA);      // e.g. 1000 mA

    // Ensure OTG and charging are disabled by default
    setCE(false);
    setOTG(false);
    // Update the RGB LED to reflect the current charger state
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
    setMuxActive(*_wire);
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
    setMuxActive(*_wire);
    bq.set_charge_current(current_mA);
}
/**
 * @brief Measures an approximate battery capacity value in mWh.
 *
 * This function estimates the battery capacity by applying a controlled 
 * load (2Ω resistor) and measuring the current difference before and after 
 * enabling the load using an analog current sensor.
 *
 * Steps:
 * - Activates the corresponding I2C mux channel.
 * - Reads the baseline current without load.
 * - Applies the load by pulling the shift-controlled load pin LOW.
 * - Measures the increased current with the load active.
 * - Calculates the power dissipated using ΔI and known resistance.
 * - Approximates energy over a short duration (~100 ms) and converts 
 *   it to milliWatt-hours (mWh).
 *
 * @return Estimated capacity in mWh. This is a relative, instantaneous 
 *         estimate useful for diagnostics or battery health checks.
 *
 * @note The precision depends on load resistor value, sensor linearity,
 *       ADC resolution, and system noise.
 */
float ChannelManager::measureCapacity() {
    // Select the appropriate I2C mux channel for this channel
    setMuxActive(*_wire);

    // === Step 1: Activate the load (pull current from battery)
    gpioManager->setShiftPin(loadPin, HIGH);  // Enable load resistor (e.g., 2Ω)
    gpioManager->applyShiftState();
    delay(50);  // Allow current to stabilize

    // === Step 2: Measure initial voltage across current sensor
    uint16_t adcRawBefore = analogRead(voltageAdcPin);
    float voltageBefore = (adcRawBefore * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
    float currentBefore = (voltageBefore - CURRENT_SENSOR_ZERO_VOLTAGE) / CURRENT_SENSOR_SENSITIVITY;

    // === Step 3: Disable load and allow signal to settle
    gpioManager->setShiftPin(loadPin, LOW);  // Disable load
    gpioManager->applyShiftState();
    delay(100);  // Wait for sensor to stabilize

    // === Step 4: Measure voltage again after load is removed
    uint16_t adcRawAfter = analogRead(voltageAdcPin);
    float voltageAfter = (adcRawAfter * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
    float currentAfter = (voltageAfter - CURRENT_SENSOR_ZERO_VOLTAGE) / CURRENT_SENSOR_SENSITIVITY;

    // === Step 5: Restore load pin to HIGH (default OFF state)
    gpioManager->setShiftPin(loadPin, HIGH);
    gpioManager->applyShiftState();

    // === Step 6: Estimate delta current and calculate discharge power
    float deltaCurrent = currentAfter - currentBefore;
    float power = deltaCurrent * deltaCurrent * LOAD_RESISTOR_OHMS;  // P = I²R
    float energy_mWs = power * 100.0f;  // Multiply by time (100ms) in milliseconds

    // === Step 7: Convert mWs to mWh (1 Wh = 3600 Ws)
    float capacity_mWh = energy_mWs / 3600.0f;

    DEBUG_PRINTF("⚡ Channel %d: CurrentBefore=%.2fmA, CurrentAfter=%.2fmA, ΔI=%.2fmA, Power=%.2fmW, Capacity=%.4fmWh\n",
                 muxChannel, currentBefore * 1000.0f, currentAfter * 1000.0f, deltaCurrent * 1000.0f, power, capacity_mWh);

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
    setMuxActive(*_wire);
    uint16_t adcRaw = analogRead(voltageAdcPin);
    float voltage = (adcRaw * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
    float capacity = measureCapacity();

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
    setMuxActive(*_wire);
    bq.enable_charger();
    setCE(true);
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
    setMuxActive(*_wire);
    bq.disable_charger();
    setCE(false);
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
    setMuxActive(*_wire);
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
    setMuxActive(*_wire);

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
