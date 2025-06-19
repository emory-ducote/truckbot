# localization

## Overview
This package implements an Extended Kalman Filter (EKF) for state estimation in ROS2. It fuses IMU and other sensor data to provide robust localization.

## Features
- EKF-based sensor fusion
- Publishes odometry as `nav_msgs/msg/Odometry`
- Subscribes to IMU and joystick data
- Written in C++ using Eigen

## Dependencies
- ROS2
- ament_cmake
- rclcpp
- sensor_msgs
- nav_msgs
- Eigen3
- fmt

## Building
Clone this package into your ROS2 workspace `src` directory and build with colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select localization
```

## Usage
Source your workspace and run the EKF node:

```bash
. install/setup.bash
ros2 run localization ekf
```

## Nodes
### ekf
- **Subscribed topics:**
  - `/imu` (`sensor_msgs/msg/Imu`): IMU data
  - `/joy` (`sensor_msgs/msg/Joy`): Joystick data
- **Published topics:**
  - `/odom` (`nav_msgs/msg/Odometry`): Estimated odometry
- **Parameters:**
  - (Add any configurable parameters here)

## Source Structure
- `include/EKF.h`: EKF class definition
- `src/EKF.cpp`: EKF implementation
- `src/EKFMiddleware.cpp`: ROS2 node for EKF and topic handling

## Maintainer
- emoryducote@gmail.com

## License
See the main LICENSE file in the repository.
