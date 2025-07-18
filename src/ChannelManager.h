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
#include "bq2589x.h"
#include "GpioManager.h"
#include "Tca9548aMux.h"

class ChannelManager {
public:
    ChannelManager(uint8_t muxChannel,
                   Tca9548aMux* mux,                          // ✅ Pass TCA Mux pointer
                   GpioManager* gpio,
                   ShiftPin rPin, ShiftPin gPin, ShiftPin bPin,
                   ShiftPin otgPin, ShiftPin cePin,
                   ShiftPin loadPin, uint8_t voltageAdcPin,
                   uint8_t intPin, uint8_t statPin, uint8_t pgPin);

    void begin();                                            ///< Initializes charger and LED for this channel

    // === Charger Control ===
    void initCharger();                                      ///< Initializes charger with default settings
    void startCharging();                                    ///< Enables charging
    void stopCharging();                                     ///< Disables charging
    bool isCharging();                                       ///< Returns whether charging is active
    void setChargingVoltage(uint16_t voltage_mV);            ///< Set charge voltage (mV)
    void setChargingCurrent(uint16_t current_mA);            ///< Set charge current (mA)

    // === Status Monitoring ===
    bool readInt();                                          ///< INT pin (interrupt, active-low)
    bool readStat();                                         ///< STAT pin (charging indicator)
    bool readPGood();                                        ///< PG pin (power good)
    ChargingStatus getStatus();                              ///< Charger and battery state report

    // === Reporting ===
    float measureCapacity();                                 ///< Quick discharge test
    void sendCapacityReport();                               ///< JSON output over Serial

    // === LED and Control ===
    void setRgbColor(bool r, bool g, bool b);                ///< RGB status indication
    void setCE(bool enabled);                                ///< CE charger enable control
    void setOTG(bool enabled);                               ///< OTG boost mode enable

    // === Accessors ===
    bq2589x& getBQ();                                        ///< Reference to charger instance
    uint8_t getMuxAddress() const;                           ///< Mux I2C address
    uint8_t getMuxChannel() const;                           ///< Logical channel number
    void updateLedFromStatus();                              ///< Reflect charger state in RGB

private:
    // === Mux & I2C ===
    uint8_t muxChannel;              ///< 0–7 channel number
    Tca9548aMux* mux = nullptr;      ///< Pointer to shared mux instance

    // === Peripherals ===
    GpioManager* gpioManager;        ///< Shared GPIO/shift driver
    bq2589x bq;                      ///< Charger driver

    // === Shift Pins ===
    ShiftPin rPin, gPin, bPin;
    ShiftPin cePin, otgPin, loadPin;

    // === Sensors and Inputs ===
    uint8_t voltageAdcPin;
    uint8_t intPin, statPin, pgPin;
};

#endif // CHANNEL_MANAGER_H
