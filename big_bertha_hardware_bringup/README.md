# Big Bertha Hardware Bringup

ROS 2 bridge between the Big Bertha quadruped and its physical hardware
(MPU6050 IMU + PCA9685 servo driver) via the Arduino UNO Q's STM32U585 MCU.

## Architecture

```
┌──────────────────────────────────────────────────────┐
│  Linux (Qualcomm QRB2210)                            │
│  ┌───────────────────────────────────────────────┐   │
│  │  hardware_bridge_node.py                       │   │
│  │  ┌────────────────┐  ┌─────────────────────┐  │   │
│  │  │ Timer @ 200 Hz  │  │ /position_controller│  │   │
│  │  │ → /imu          │  │ /commands subscriber│  │   │
│  │  └───────┬────────┘  └─────────┬───────────┘  │   │
│  └──────────┼─────────────────────┼──────────────┘   │
└─────────────┼─────────────────────┼──────────────────┘
              │  Bridge (serial RPC) │
┌─────────────┼─────────────────────┼──────────────────┐
│  STM32U585  │                     │                  │
│  ┌──────────▼─────────┐  ┌────────▼─────────┐       │
│  │  read_imu callback  │  │ set_servos callback│      │
│  │  MPU6050 via I2C    │  │ PCA9685 via I2C   │       │
│  └────────────────────┘  └───────────────────┘       │
└──────────────────────────────────────────────────────┘
```

The STM32U585 (Arduino UNO Q MCU) owns the physical I2C bus with both the
MPU6050 (addr `0x68`) and PCA9685 (addr `0x40`). The Linux side communicates
over the Arduino Router Bridge (internal serial RPC) — no direct I2C access
from Linux.

## Bridge API

| Call | Args | STM32 returns |
|---|---|---|
| `read_imu` | — | `"gx gy gz ax ay az"` — 6 floats, rad/s + m/s² |
| `set_servos` | `"p0 p1 p2 ... p11"` | `"ok"` — 12 floats as pulse µs, clipped to [0, 2500] |
| `ping` | — | `"ok"` |

The STM32 handles raw int16 → SI conversion internally (gyro: °/s → rad/s,
accel: g → m/s²) so the Linux side only passes floats through.

## Project structure

```
big_bertha_hardware_bringup/                      # ROS 2 package
├── big_bertha_hardware_bringup/
│   └── hardware_bridge_node.py                    # ROS 2 node
├── config/hardware_bridge.yaml                    # config
├── launch/hardware_bridge.launch.py
├── package.xml
    └── firmware/
        └── big_bertha_hardware/                       # App Lab project
            ├── app.yaml                                # app manifest
            ├── python/
            │   ├── main.py                             # minimal background loop (required by App Lab)
            │   └── requirements.txt
            └── sketch/
                ├── sketch.ino                          # STM32 firmware
                ├── sketch.yaml                         # FQBN: arduino:zephyr:unoq
                ├── mpu6050.h / .cpp                    # MPU6050 I2C driver
                └── pca9685.h / .cpp                    # PCA9685 I2C driver
```

## Usage

### 1. Flash the STM32 firmware

The firmware runs on the STM32U585 MCU. Deploy via `arduino-app-cli`:

```bash
# Copy the App Lab project to the UNO Q
scp -r firmware/big_bertha_hardware \
  arduino@<UNO_Q_IP>:~/ArduinoApps/

# SSH into the UNO Q
ssh arduino@<UNO_Q_IP>

# Flash and run
arduino-app-cli app start ~/ArduinoApps/big_bertha_hardware

# View logs (optional)
arduino-app-cli app logs ~/ArduinoApps/big_bertha_hardware

# Stop the app
arduino-app-cli app stop ~/ArduinoApps/big_bertha_hardware
```

Alternatively, flash via USB from a host PC with `arduino-cli`:

```bash
arduino-cli compile --fqbn arduino:zephyr:unoq firmware/big_bertha_hardware/sketch
arduino-cli upload --fqbn arduino:zephyr:unoq firmware/big_bertha_hardware/sketch
```

### 2. Run the ROS 2 bridge node

On the UNO Q's Linux side:

```bash
ros2 launch big_bertha_hardware_bringup hardware_bridge.launch.py
```

### 3. Verify

```bash
# Check /imu is publishing
ros2 topic echo /imu --once

# Ping the hardware
ros2 service call /hardware/ping std_srvs/srv/Trigger

# Check node is alive
ros2 node list | grep hardware_bridge
```

## Topics

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `/imu` | `sensor_msgs/Imu` | Published | 200 Hz |
| `/position_controller/commands` | `std_msgs/Float64MultiArray` | Subscribed | on receipt |
| `/hardware/ping` | `std_srvs/Trigger` | Service | on demand |

## Dependencies

- **Firmware:** `Arduino.h` + `Wire.h` + `Arduino_RouterBridge.h` only
- **Linux side:** `arduino.app_utils` (pre-installed on UNO Q) or direct serial

## Hardware

| Component | Interface | Address | Pins |
|---|---|---|---|
| Arduino UNO Q | — | — | — |
| MPU6050 IMU | I2C | `0x68` | SDA → A4, SCL → A5 |
| PCA9685 servo driver | I2C | `0x40` | SDA → A4, SCL → A5 |
| MG995 servos ×12 | PWM (PCA9685 channels 0–11) | — | PCA9685 output pins |
