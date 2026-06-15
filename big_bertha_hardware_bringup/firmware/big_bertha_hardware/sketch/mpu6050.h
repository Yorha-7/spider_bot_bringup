#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

class MPU6050 {
public:
  bool begin(TwoWire &wire = Wire);
  bool read(float &gx, float &gy, float &gz, float &ax, float &ay, float &az);

private:
  TwoWire *_wire;
  uint8_t _addr = MPU6050_ADDR;
  float _gyroScale = 250.0f / 32768.0f * (PI / 180.0f);
  float _accelScale = 2.0f / 32768.0f * 9.80665f;

  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};

#endif
