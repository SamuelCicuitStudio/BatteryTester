#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include <Wire.h>
#include "ChannelManager.h"
#include "SwitchManager.h"
#include "GpioManager.h"
#include "Tca9548aMux.h"  

class DeviceManager {
public:
    explicit DeviceManager(TwoWire* i2cBus);  // Constructor takes I2C bus

    void begin();               // Initializes hardware and channels
    void updateAllChannels();   // Calls sendCapacityReport on all channels

private:
    TwoWire* wire;                  // Pointer to I2C interface (Wire or Wire1, etc.)
    GpioManager* gpio;              // GPIO and shift register manager
    Tca9548aMux* mux;               // ✅ Pointer to TCA9548A I2C multiplexer

    ChannelManager* ch1;
    ChannelManager* ch2;
    ChannelManager* ch3;
    ChannelManager* ch4;

    SwitchManager* switchManager;   // Tap-based trigger manager
};

#endif // DEVICE_MANAGER_H
