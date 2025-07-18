#include "DeviceManager.h"

// Constructor: Initializes all channels and the SwitchManager using initializer list
DeviceManager::DeviceManager(TwoWire* i2cBus)
    : wire(i2cBus),
      ch1(new ChannelManager(0, &gpio, SHIFT_R1, SHIFT_G1, SHIFT_B1, SHIFT_OTG1, SHIFT_CE1, SHIFT_LD01, VOUTCRR1_PIN, INT1_PIN, STAT1_PIN, PG1_PIN)),
      ch2(new ChannelManager(1, &gpio, SHIFT_R2, SHIFT_G2, SHIFT_B2, SHIFT_OTG2, SHIFT_CE2, SHIFT_LD02, VOUTCRR2_PIN, INT2_PIN, STAT2_PIN, PG2_PIN)),
      ch3(new ChannelManager(2, &gpio, SHIFT_R3, SHIFT_G3, SHIFT_B3, SHIFT_OTG3, SHIFT_CE3, SHIFT_LD03, VOUTCRR3_PIN, INT3_PIN, STAT3_PIN, PG3_PIN)),
      ch4(new ChannelManager(3, &gpio, SHIFT_R4, SHIFT_G4, SHIFT_B4, SHIFT_OTG4, SHIFT_CE4, SHIFT_LD04, VOUTCRR4_PIN, INT4_PIN, STAT4_PIN, PG4_PIN)),
      switchManager(ch1, ch2, ch3, ch4)  // ✅ Correct initialization of SwitchManager
{
}

// Begin all peripherals
void DeviceManager::begin() {
    DEBUG_PRINTLN("===================================================");
    DEBUG_PRINTLN("#               Starting Device Manager           #");
    DEBUG_PRINTLN("===================================================");

    gpio.begin();      // Initialize GPIO manager
    wire->begin();     // Initialize I2C bus

    ch1->begin(*wire);
    ch2->begin(*wire);
    ch3->begin(*wire);
    ch4->begin(*wire);

    switchManager.TapDetect();  // Start input detection task
}

// Updates capacity and voltage reporting for all channels
void DeviceManager::updateAllChannels() {
    ch1->sendCapacityReport();
    ch2->sendCapacityReport();
    ch3->sendCapacityReport();
    ch4->sendCapacityReport();
}
