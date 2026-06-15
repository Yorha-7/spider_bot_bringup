/**
 * Big Bertha Hardware Bridge — STM32U585 firmware
 *
 * Reads MPU6050 IMU and controls PCA9685 servo driver over I2C.
 * Communicates with the Linux side via Arduino Router Bridge.
 *
 * Bridge API:
 *   read_imu    →  "gx gy gz ax ay az"    6 floats, rad/s + m/s²
 *   set_servos  ←  "p0 p1 ... p11"        12 floats, pulse µs
 *   ping        →  "ok"
 *
 * Wiring:
 *   MPU6050 SDA → A4        PCA9685 SDA → A4
 *   MPU6050 SCL → A5        PCA9685 SCL → A5
 *   (shared I2C bus, addrs 0x68 and 0x40)
 */

#include "Arduino_RouterBridge.h"
#include "mpu6050.h"
#include "pca9685.h"

<<<<<<< HEAD
#define IMU_INTERVAL_US 5000
#define NUM_SERVOS 12
#define SERVO_FREQ 50
=======
#define IMU_INTERVAL_US  5000
#define NUM_SERVOS       12
#define SERVO_FREQ       50
>>>>>>> fd42369 (feat: MPU6050 and PAC9685 hardware interface for big_bertha)

MPU6050 mpu;
PCA9685 pca;

static float _gx, _gy, _gz, _ax, _ay, _az;
static uint32_t _lastImuUs = 0;

static String readImuCallback() {
<<<<<<< HEAD
  char buf[96];
  snprintf(buf, sizeof(buf), "%.6f %.6f %.6f %.6f %.6f %.6f", (double)_gx,
           (double)_gy, (double)_gz, (double)_ax, (double)_ay, (double)_az);
  return String(buf);
}

static void setServosCallback(String data) {
  uint16_t pulses[NUM_SERVOS];
  int count = 0;
  int start = 0;

  for (unsigned int i = 0; i <= data.length(); i++) {
    if (i == data.length() || data[i] == ' ' || data[i] == ',') {
      if (i > (unsigned int)start) {
        String tok = data.substring(start, i);
        if (count < NUM_SERVOS) {
          float val = tok.toFloat();
          if (val < 0)
            val = 0;
          if (val > 2500)
            val = 2500;
          pulses[count++] = (uint16_t)val;
        }
      }
      start = i + 1;
    }
  }

  pca.setAllPulses(pulses, count);
}

static String pingCallback() { return "ok"; }

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  if (!mpu.begin()) {
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }

  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);

  Bridge.begin();
  Bridge.provide("read_imu", readImuCallback);
  Bridge.provide("set_servos", setServosCallback);
  Bridge.provide("ping", pingCallback);

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  uint32_t now = micros();
  if (now - _lastImuUs >= IMU_INTERVAL_US) {
    _lastImuUs = now;
    mpu.read(_gx, _gy, _gz, _ax, _ay, _az);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
=======
    char buf[96];
    snprintf(buf, sizeof(buf),
             "%.6f %.6f %.6f %.6f %.6f %.6f",
             (double)_gx, (double)_gy, (double)_gz,
             (double)_ax, (double)_ay, (double)_az);
    return String(buf);
}

static void setServosCallback(String data) {
    uint16_t pulses[NUM_SERVOS];
    int count = 0;
    int start = 0;

    for (unsigned int i = 0; i <= data.length(); i++) {
        if (i == data.length() || data[i] == ' ' || data[i] == ',') {
            if (i > (unsigned int)start) {
                String tok = data.substring(start, i);
                if (count < NUM_SERVOS) {
                    float val = tok.toFloat();
                    if (val < 0) val = 0;
                    if (val > 2500) val = 2500;
                    pulses[count++] = (uint16_t)val;
                }
            }
            start = i + 1;
        }
    }

    pca.setAllPulses(pulses, count);
}

static String pingCallback() {
    return "ok";
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    if (!mpu.begin()) {
        while (true) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }

    pca.begin();
    pca.setPWMFreq(SERVO_FREQ);

    Bridge.begin();
    Bridge.provide("read_imu", readImuCallback);
    Bridge.provide("set_servos", setServosCallback);
    Bridge.provide("ping", pingCallback);

    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    uint32_t now = micros();
    if (now - _lastImuUs >= IMU_INTERVAL_US) {
        _lastImuUs = now;
        mpu.read(_gx, _gy, _gz, _ax, _ay, _az);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
>>>>>>> fd42369 (feat: MPU6050 and PAC9685 hardware interface for big_bertha)
}
