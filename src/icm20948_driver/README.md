# icm20948_driver

## Overview
ROS2 driver for the ICM-20948 9-axis IMU. Reads accelerometer, gyroscope, and magnetometer data from the sensor over I2C and publishes it at 100 Hz as `sensor_msgs/Imu`. Performs automatic sensor calibration on startup.

## Features
- ICM-20948 initialization and configuration over I2C (via lgpio)
- Automatic bias calibration for accelerometer, gyroscope, and magnetometer on startup (40-sample average)
- Publishes at 100 Hz
- Graceful shutdown with sensor reset

## Dependencies
- ROS2
- ament_cmake
- rclcpp
- sensor_msgs
- Eigen3
- lgpio
- fmt

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select icm20948_driver
```

## Usage

```bash
. install/setup.bash
ros2 run icm20948_driver imu
```

Or with the launch file:

```bash
ros2 launch icm20948_driver imu.launch.py
```

## Nodes

### imu
Reads all three sensor axes from the ICM-20948 and publishes them as a single `Imu` message at 100 Hz. Magnetometer data is packed into the `orientation` field (x/y/z) since `sensor_msgs/Imu` has no dedicated magnetometer field.

- **Subscribed topics:** None
- **Published topics:**
  - `/imu` (`sensor_msgs/msg/Imu`): accelerometer (m/s²), gyroscope (rad/s), and magnetometer data at 100 Hz
- **Parameters:**
  - `i2c_device` (int, default: `0x68`): I2C device address of the ICM-20948

## Source Structure
- `include/rpi5ICM20948.h`: ICM-20948 register map and driver class
- `src/rpi5ICM20948.cpp`: I2C communication and sensor initialization
- `src/IMUMiddleware.cpp`: ROS2 node that publishes IMU data
- `config/imu.yaml`: default parameters
- `launch/imu.launch.py`: launch file

## Maintainer
- emoryducote@gmail.com

## License
MIT
