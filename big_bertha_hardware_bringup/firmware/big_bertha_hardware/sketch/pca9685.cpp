#include "pca9685.h"

bool PCA9685::begin(TwoWire &wire) {
    _wire = &wire;
    writeReg(PCA9685_MODE1, 0x00);
    delay(10);
    return true;
}

void PCA9685::setPWMFreq(uint16_t freq) {
    if (freq == 0) return;
    _periodUs = 1000000 / freq;

    uint16_t prescale = (uint16_t)(25000000.0f / (4096.0f * freq) - 0.5f);

    uint8_t oldMode;
    _wire->beginTransmission(_addr);
    _wire->write(PCA9685_MODE1);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);
    oldMode = _wire->read();

    writeReg(PCA9685_MODE1, (oldMode & 0x7F) | 0x10);
    writeReg(PCA9685_PRESCALE, prescale);
    writeReg(PCA9685_MODE1, oldMode);
    delay(10);
    writeReg(PCA9685_MODE1, oldMode | 0xA1);
}

void PCA9685::setPulse(uint8_t channel, uint16_t pulseUs) {
    if (pulseUs > _periodUs) pulseUs = _periodUs;
    uint16_t off = (uint32_t)pulseUs * 4096 / _periodUs;
    if (off > 4095) off = 4095;

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(0);
    _wire->write(0);
    _wire->write(off & 0xFF);
    _wire->write((off >> 8) & 0xFF);
    _wire->endTransmission();
}

void PCA9685::setAllPulses(const uint16_t *pulseUs, uint8_t count) {
    for (uint8_t i = 0; i < count && i < PCA9685_CHANNELS; i++)
        setPulse(i, pulseUs[i]);
}

void PCA9685::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}
