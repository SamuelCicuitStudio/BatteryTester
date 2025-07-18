/**************************************************************
 *  Author      : Tshibangu Samuel
 *  Role        : Freelance Embedded Systems Engineer
 *  Expertise   : Secure IoT Systems, Embedded C++, RTOS, Control Logic
 *  Contact     : tshibsamuel47@gmail.com
 *  Portfolio   : https://www.freelancer.com/u/tshibsamuel477
 *  Phone       : +216 54 429 793
 **************************************************************/
#ifndef TCA9548A_MUX_H
#define TCA9548A_MUX_H

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

class Tca9548aMux {
public:
    Tca9548aMux(uint8_t address = TCA9548A_ADDR);  // default I2C address
    void begin(TwoWire &wirePort = Wire);
    void reset();
    bool selectChannel(uint8_t channel);  // 0-7
    void disableAll();
    bool isChannelEnabled(uint8_t channel);
    uint8_t getCurrentSelection();

private:
    uint8_t _address;
    TwoWire* _wire;
    uint8_t _lastValue = 0x00;

    void writeMux(uint8_t val);
};

#endif
