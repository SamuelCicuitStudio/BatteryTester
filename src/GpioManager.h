/**************************************************************
 *  Author      : Tshibangu Samuel
 *  Role        : Freelance Embedded Systems Engineer
 *  Expertise   : Secure IoT Systems, Embedded C++, RTOS, Control Logic
 *  Contact     : tshibsamuel47@gmail.com
 *  Portfolio   : https://www.freelancer.com/u/tshibsamuel477
 *  Phone       : +216 54 429 793
 **************************************************************/
#ifndef GPIOMANAGER_H
#define GPIOMANAGER_H

#include "Config.h"

class GpioManager {
public:
    void begin();
    void setShiftPin(ShiftPin pin, bool level);
    void applyShiftState();  // push internal state to hardware
    void resetShiftRegisters();
    void enableShiftRegisterOutput(bool enable);

private:
    uint32_t shiftState = 0;  // internal 24-bit shadow register
    void shiftOutState();
};

#endif
