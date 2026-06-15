#ifndef PCA9685_H
#define PCA9685_H

#include <Arduino.h>
#include <Wire.h>

<<<<<<< HEAD
#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_CHANNELS 16

    class PCA9685 {
public:
  bool begin(TwoWire &wire = Wire);
  void setPulse(uint8_t channel, uint16_t pulseUs);
  void setAllPulses(const uint16_t *pulseUs, uint8_t count);
  void setPWMFreq(uint16_t freq);

private:
  TwoWire *_wire;
  uint8_t _addr = PCA9685_ADDR;
  uint16_t _periodUs = 20000;

  void writeReg(uint8_t reg, uint8_t val);
=======
#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_CHANNELS 16

class PCA9685 {
public:
  bool begin(TwoWire &wire = Wire);
  void setPulse(uint8_t channel, uint16_t pulseUs);
  void setAllPulses(const uint16_t *pulseUs, uint8_t count);
  void setPWMFreq(uint16_t freq);

private:
  TwoWire *_wire;
  uint8_t _addr = PCA9685_ADDR;
  uint16_t _periodUs = 20000;

  void writeReg(uint8_t reg, uint8_t val);
>>>>>>> fd42369 (feat: MPU6050 and PAC9685 hardware interface for big_bertha)
};

#endif
