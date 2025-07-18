#include "Tca9548aMux.h"

Tca9548aMux::Tca9548aMux(uint8_t address)
    : _address(address), _wire(&Wire) {}

void Tca9548aMux::begin(TwoWire &wirePort) {
    _wire = &wirePort;

    pinMode(TCA_RESET_PIN, OUTPUT);
    digitalWrite(TCA_RESET_PIN, HIGH);  // keep mux active

    // Start I2C
    _wire->begin(I2C_SDA_PIN, I2C_SCL_PIN);
}

void Tca9548aMux::reset() {
    digitalWrite(TCA_RESET_PIN, LOW);
    delay(5);
    digitalWrite(TCA_RESET_PIN, HIGH);
    delay(5);
}

bool Tca9548aMux::selectChannel(uint8_t channel) {
    if (channel > 7) return false;

    uint8_t val = 1 << channel;
    writeMux(val);
    _lastValue = val;
    return true;
}

void Tca9548aMux::disableAll() {
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
