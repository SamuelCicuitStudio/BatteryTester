/**************************************************************
 *  Author      : Tshibangu Samuel
 *  Role        : Freelance Embedded Systems Engineer
 *  Expertise   : Secure IoT Systems, Embedded C++, RTOS, Control Logic
 *  Contact     : tshibsamuel47@gmail.com
 *  Portfolio   : https://www.freelancer.com/u/tshibsamuel477
 *  Phone       : +216 54 429 793
 **************************************************************/
#ifndef CHANNEL_MANAGER_H
#define CHANNEL_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include "bq2589x.h"
#include "GpioManager.h"


class ChannelManager {
public:

    ChannelManager(uint8_t muxChannel,
                   GpioManager* gpio,
                   ShiftPin rPin, ShiftPin gPin, ShiftPin bPin,
                   ShiftPin otgPin, ShiftPin cePin,
                   ShiftPin loadPin, uint8_t voltageAdcPin,
                   uint8_t intPin, uint8_t statPin, uint8_t pgPin);

    void begin(TwoWire& wire);                                ///< Initializes the I2C bus and charger for this channel

    // === Charger Control ===

    void initCharger();                                       ///< Initializes charger configuration with default settings
    void startCharging();                                     ///< Enables the charger and begins battery charging
    void stopCharging();                                      ///< Disables the charger and stops charging process
    bool isCharging();                                        ///< Returns true if charger is currently enabled
    void setChargingVoltage(uint16_t voltage_mV);             ///< Sets the target charge voltage in millivolts
    void setChargingCurrent(uint16_t current_mA);             ///< Sets the target charge current in milliamps

    // === Status Monitoring ===

    bool readInt();                                           ///< Reads the state of the INT pin (active-low)
    bool readStat();                                          ///< Reads the state of the STAT pin (charging status)
    bool readPGood();                                         ///< Reads the state of the PG (Power Good) pin
    ChargingStatus getStatus();                               ///< Returns a snapshot of charger state and measurements

    // === Reporting ===

    float measureCapacity();                                  ///< Performs a brief discharge to estimate battery capacity
    void sendCapacityReport();                                ///< Sends voltage and capacity as JSON to Serial

    void setRgbColor(bool r, bool g, bool b);                 ///< Sets RGB LED state using shift register pins
    void setCE(bool enabled);                                 ///< Enables/disables the CE (charger enable) line
    void setOTG(bool enabled);                                ///< Enables/disables OTG (boost output) function

    // === Accessors ===

    void setMuxActive(TwoWire& wire);                         ///< Activates the correct channel on the TCA9548A mux
    bq2589x& getBQ();                                         ///< Returns a reference to the BQ2589x driver
    uint8_t getMuxAddress() const;                            ///< Returns the I2C address of the TCA mux
    uint8_t getMuxChannel() const;                            ///< Returns the active mux channel bitmask
    void updateLedFromStatus();                               ///< Updates RGB LED color based on charging status



private:
    // === Mux & I2C ===

    uint8_t tcaAddress;        ///< I2C address of the TCA9548A multiplexer
    uint8_t muxChannel;        ///< Logical channel number (0–7)
    uint8_t muxChannelBit;     ///< Bitmask to activate the mux channel
    TwoWire* _wire = nullptr;  ///< Pointer to active I2C bus

    // === Peripheral Drivers ===

    GpioManager* gpioManager;  ///< Shared GPIO/shift register manager
    bq2589x bq;                ///< Driver instance for BQ2589x charger

    // === Shift Register Pins ===

    ShiftPin rPin;             ///< Red LED shift pin
    ShiftPin gPin;             ///< Green LED shift pin
    ShiftPin bPin;             ///< Blue LED shift pin
    ShiftPin cePin;            ///< CE (Charger Enable) shift pin
    ShiftPin otgPin;           ///< OTG (Boost Mode Enable) shift pin
    ShiftPin loadPin;          ///< Load enable shift pin (for discharging)

    // === Analog Inputs ===
    uint8_t voltageAdcPin;     ///< ADC pin connected to current sensor output

    // === Digital Status Inputs ===
    uint8_t intPin;            ///< INT pin from charger (interrupt)
    uint8_t statPin;           ///< STAT pin from charger (status indicator)
    uint8_t pgPin;             ///< PG (Power Good) pin from charger
};

#endif // CHANNEL_MANAGER_H