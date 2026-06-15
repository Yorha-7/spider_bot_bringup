#include "mpu6050.h"

bool MPU6050::begin(TwoWire &wire) {
    _wire = &wire;
    _wire->begin();

    uint8_t whoami;
    if (!readRegs(MPU6050_WHO_AM_I, &whoami, 1))
        return false;
    if (whoami != 0x68)
        return false;

    writeReg(MPU6050_PWR_MGMT_1, 0x00);
    delay(10);

    writeReg(MPU6050_SMPLRT_DIV, 0x00);
    writeReg(MPU6050_CONFIG, 0x00);
    writeReg(MPU6050_GYRO_CONFIG, 0x00);
    writeReg(MPU6050_ACCEL_CONFIG, 0x00);

    return true;
}

bool MPU6050::read(float &gx, float &gy, float &gz,
                    float &ax, float &ay, float &az) {
    uint8_t buf[14];
    if (!readRegs(MPU6050_ACCEL_XOUT_H, buf, 14))
        return false;

    int16_t raw_ax = (int16_t)(buf[0] << 8 | buf[1]);
    int16_t raw_ay = (int16_t)(buf[2] << 8 | buf[3]);
    int16_t raw_az = (int16_t)(buf[4] << 8 | buf[5]);
    int16_t raw_gx = (int16_t)(buf[8] << 8 | buf[9]);
    int16_t raw_gy = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t raw_gz = (int16_t)(buf[12] << 8 | buf[13]);

    gx = raw_gx * _gyroScale;
    gy = raw_gy * _gyroScale;
    gz = raw_gz * _gyroScale;

    ax = raw_ax * _accelScale;
    ay = raw_ay * _accelScale;
    az = raw_az * _accelScale;

    return true;
}

bool MPU6050::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission() == 0;
}

bool MPU6050::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0)
        return false;
    if (_wire->requestFrom(_addr, len) != len)
        return false;
    for (uint8_t i = 0; i < len; i++)
        buf[i] = _wire->read();
    return true;
}
