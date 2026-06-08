# Anveshak Rover — Embedded Firmware

Micro-ROS firmware for the embedded systems of Team Anveshak's rover, running on ESP32-S3 targets over ROS 2 (Humble) serial transport.

---

## Repository Structure

| File | Description |
|---|---|
| `arm.ino` | 6-DOF robotic arm controller with encoder feedback and PID |
| `drive.ino` | 6-motor drive controller with watchdog and RGB status |
| `GPS_IMU_Microros.ino` | GPS + IMU sensor node publishing to ROS 2 topics |

---

## arm.ino — Robotic Arm Controller

Controls 6 joints: **Base, Shoulder, Elbow, Bevel 1, Bevel 2, Gripper**

### GPIO Pins

| Joint | PWM Pin | DIR Pin | Enc A | Enc B |
|---|---|---|---|---|
| Base | 2 | 1 | 39 | 40 |
| Shoulder | 41 | 42 | 48 | 38 |
| Elbow | 6 | 7 | 21 | 47 |
| Bevel 1 | 4 | 5 | 14 | 13 |
| Bevel 2 | 15 | 16 | 12 | 11 |
| Gripper | 17 | 18 | 10 | 9 |

### ROS 2 Topics

| Topic | Type | Direction |
|---|---|---|
| `/arm_target_angles` | `Float32MultiArray` | Subscribe |
| `/input_space` | `Bool` | Subscribe |
| `/arm_state` | `Bool` | Subscribe |
| `/arm_target_echo` | `Float32MultiArray` | Publish |
| `/enc_counts` | `Int32MultiArray` | Publish |
| `/enc_angles` | `Float32MultiArray` | Publish |

### Control Modes

Determined by the `/input_space` and `/arm_state` flags:

- `/arm_state = true` → All 6 joints run angle PID
- `/input_space = true`, `/arm_state = false` → Joints 0–2 run angle PID, joints 3–5 run direct PWM
- Both `false` → All 6 joints run direct PWM

PID runs at **50 Hz** (20 ms timer). Default gains: `Kp = 2`, `Ki = 0`, `Kd = 0` per joint.

---

## drive.ino — Drive Controller

Controls 6 drive motors with a watchdog safety stop.

### GPIO Pins

| Motor | PWM Pin | DIR Pin |
|---|---|---|
| 1 | 36 | 37 |
| 2 | 38 | 39 |
| 3 | 40 | 41 |
| 4 | 42 | 2 |
| 5 | 4 | 5 |
| 6 | 6 | 7 |

**Indicator:** GPIO 48 → Red LED

### ROS 2 Topics

| Topic | Type | Direction |
|---|---|---|
| `/drive_pwm` | `Int32MultiArray` | Subscribe |
| `/state` | `Bool` | Subscribe |
| `/drive_pwm_echo` | `Int32MultiArray` | Publish |
| `/state_echo` | `Bool` | Publish |
| `/gpio_state` | `Bool` | Publish |

### Notes

- PWM input range: **−127 to 127** (negative reverses direction)
- Watchdog timeout: **1 second** — motors stop if no `/drive_pwm` message is received
- RGB LED status: Blue = waiting for agent, Green = normal, Yellow = watchdog triggered, Red = transport error

---

## GPS_IMU_Microros.ino — Sensor Node

Reads GPS and IMU data and publishes to ROS 2 at **2 Hz** (500 ms timer).

### GPIO Pins

| Bus | SDA | SCL | Device |
|---|---|---|---|
| I2C (GPS) | 17 | 16 | u-blox GNSS |
| I2C (IMU) | 35 | 36 | MPU6050 |

### ROS 2 Topics

| Topic | Type | Direction |
|---|---|---|
| `gps/fix` | `sensor_msgs/NavSatFix` | Publish |
| `imu/data_raw` | `sensor_msgs/Imu` | Publish |

### IMU Configuration

- Accelerometer range: ±8 G
- Gyro range: ±500 °/s
- Filter bandwidth: 21 Hz

---

## Dependencies

- [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino)
- [SparkFun u-blox GNSS v3](https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3)
- [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050)
- ROS 2 Humble

## Transport

All nodes use **micro-ROS serial transport** (`set_microros_transports()`). Ensure the micro-ROS agent is running on the host before powering the ESP32.

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
