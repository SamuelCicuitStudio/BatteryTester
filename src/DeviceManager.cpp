#include "DeviceManager.h"
#include "Config.h"  // For SHIFT pins and voltage ADC pins

DeviceManager::DeviceManager(TwoWire* i2cBus)
    : wire(i2cBus),
      gpio(nullptr),
      mux(nullptr),
      ch1(nullptr), ch2(nullptr), ch3(nullptr), ch4(nullptr),
      switchManager(nullptr) {}

void DeviceManager::begin() {
    // ✅ Allocate and initialize GPIO manager
    gpio = new GpioManager();
    gpio->begin();

    // ✅ Allocate and initialize TCA9548A multiplexer
    mux = new Tca9548aMux(wire, TCA9548A_ADDR);
    mux->begin();
    mux->reset();

    // ✅ Create all ChannelManager instances
    ch4 = new ChannelManager(0, mux, gpio,
        SHIFT_R1, SHIFT_G1, SHIFT_B1,
        SHIFT_OTG1, SHIFT_CE1, SHIFT_LD01,
        VOUTCRR1_PIN, INT1_PIN, STAT1_PIN, PG1_PIN);

    ch3 = new ChannelManager(1, mux, gpio,
        SHIFT_R2, SHIFT_G2, SHIFT_B2,
        SHIFT_OTG2, SHIFT_CE2, SHIFT_LD02,
        VOUTCRR2_PIN, INT2_PIN, STAT2_PIN, PG2_PIN);

    ch2 = new ChannelManager(2, mux, gpio,
        SHIFT_R3, SHIFT_G3, SHIFT_B3,
        SHIFT_OTG3, SHIFT_CE3, SHIFT_LD03,
        VOUTCRR3_PIN, INT3_PIN, STAT3_PIN, PG3_PIN);

    ch1 = new ChannelManager(3, mux, gpio,
        SHIFT_R4, SHIFT_G4, SHIFT_B4,
        SHIFT_OTG4, SHIFT_CE4, SHIFT_LD04,
        VOUTCRR4_PIN, INT4_PIN, STAT4_PIN, PG4_PIN);

    // ✅ Begin all channels
    ch1->begin();
    ch2->begin();
    ch3->begin();
    ch4->begin();

    // ✅ Create and start SwitchManager
    switchManager = new SwitchManager(ch1, ch2, ch3, ch4);
    switchManager->TapDetect();
}

void DeviceManager::updateAllChannels() {
    if (ch1) ch1->sendCapacityReport();
    if (ch2) ch2->sendCapacityReport();
    if (ch3) ch3->sendCapacityReport();
    if (ch4) ch4->sendCapacityReport();
}
