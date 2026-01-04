# icm20948_driver

## Overview
This package provides a ROS2 driver node for the ICM-20948 IMU sensor. It publishes IMU data to ROS2 topics for use in localization and control.

## Features
- ICM-20948 initialization and configuration
- Publishes IMU data (accelerometer, gyroscope, magnetometer) as `sensor_msgs/msg/Imu`
- Written in C++ for performance

## Dependencies
- ROS2 
- ament_cmake
- rclcpp
- sensor_msgs
- Eigen3
- lgpio
- fmt

## Building
Clone this package into your ROS2 workspace `src` directory and build with colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select icm20948_driver
```

## Usage
Source your workspace and run the IMU node:

```bash
. install/setup.bash
ros2 run icm20948_driver imu
```

## Nodes
### imu
- **Subscribed topics:** None
- **Published topics:**
  - `/imu` (`sensor_msgs/msg/Imu`): Publishes IMU data
- **Parameters:**
  - (Add any configurable parameters here)

## Source Structure
- `include/rpi5ICM20948.h`: IMU register definitions and class
- `src/rpi5ICM20948.cpp`: IMU hardware interface implementation
- `src/IMUMiddleware.cpp`: ROS2 node for publishing IMU data

## Maintainer
- emoryducote@gmail.com

## License
See the main LICENSE file in the repository.
