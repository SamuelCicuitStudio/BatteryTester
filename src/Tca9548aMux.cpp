#include "Tca9548aMux.h"

Tca9548aMux::Tca9548aMux(TwoWire* wire, uint8_t address)
    : _address(address), _wire(wire) {}

void Tca9548aMux::begin() {
    // Set A0, A1, A2 as OUTPUT and LOW to use address 0x70 (default)
    pinMode(TCA_A0_PIN, OUTPUT);
    pinMode(TCA_A1_PIN, OUTPUT);
    pinMode(TCA_A2_PIN, OUTPUT);
    digitalWrite(TCA_A0_PIN, LOW);
    digitalWrite(TCA_A1_PIN, LOW);
    digitalWrite(TCA_A2_PIN, LOW);

    // Prepare and apply reset before starting I2C
    pinMode(TCA_RESET_PIN, OUTPUT);
    digitalWrite(TCA_RESET_PIN, LOW);
    delay(5);
    digitalWrite(TCA_RESET_PIN, HIGH);
    delay(5);

    // Begin I2C bus on specified pins
   // _wire->begin(I2C_SDA_PIN, I2C_SCL_PIN);

    DEBUG_PRINTF("🔌 TCA9548A Mux initialized on address 0x%02X\n", _address);
}


void Tca9548aMux::reset() {
    DEBUG_PRINTLN("🔁 Resetting TCA9548A Mux...");
    digitalWrite(TCA_RESET_PIN, LOW);
    delay(5);
    digitalWrite(TCA_RESET_PIN, HIGH);
    delay(5);
    _lastValue = 0;
    DEBUG_PRINTLN("✓ Mux reset complete.");
}

bool Tca9548aMux::selectChannel(uint8_t channel) {
    if (channel > 7) {
        DEBUG_PRINTF("❌ Invalid Mux channel: %d\n", channel);
        return false;
    }

    uint8_t val = 1 << channel;
    if (_lastValue == val) {
        return true;  // Already selected
    }

    DEBUG_PRINTF("➡️ Selecting TCA9548A channel %d (0x%02X)\n", channel, val);
    writeMux(val);
    _lastValue = val;
    return true;
}

void Tca9548aMux::disableAll() {
    DEBUG_PRINTLN("🚫 Disabling all TCA9548A channels");
    writeMux(0x00);
    _lastValue = 0;
}

bool Tca9548aMux::isChannelEnabled(uint8_t channel) {
    if (channel > 7) return false;
    return (_lastValue & (1 << channel));
}

uint8_t Tca9548aMux::getCurrentSelection() {
    return _lastValue;
}

void Tca9548aMux::writeMux(uint8_t val) {
    _wire->beginTransmission(_address);
    _wire->write(val);
    _wire->endTransmission();
}

uint8_t Tca9548aMux::getAddress() const {
    return _address;
}

