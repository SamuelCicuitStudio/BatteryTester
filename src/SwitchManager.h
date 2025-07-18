/**************************************************************
 *  Author      : Tshibangu Samuel
 *  Role        : Freelance Embedded Systems Engineer
 *  Expertise   : Secure IoT Systems, Embedded C++, RTOS, Control Logic
 *  Contact     : tshibsamuel47@gmail.com
 *  Portfolio   : https://www.freelancer.com/u/tshibsamuel477
 *  Phone       : +216 54 429 793
 **************************************************************/

#ifndef SWITCH_MANAGER_H
#define SWITCH_MANAGER_H

#include "Config.h"
#include "ChannelManager.h"

#define BUTTON_PIN            USR_SWITCH_PIN
#define TAP_WINDOW_MS         1200
#define HOLD_THRESHOLD_MS     3000

class SwitchManager {
public:
    // Constructor accepts 4 channels
    SwitchManager(ChannelManager* ch1, ChannelManager* ch2,
                  ChannelManager* ch3, ChannelManager* ch4);

    void detectTapOrHold();
    void TapDetect();
    static void SwitchTask(void* pvParameters);
    static SwitchManager* instance;

private:
    ChannelManager* channel1;
    ChannelManager* channel2;
    ChannelManager* channel3;
    ChannelManager* channel4;
};

#endif // SWITCH_MANAGER_H
