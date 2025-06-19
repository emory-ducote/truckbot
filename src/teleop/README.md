f# teleop

## Overview
This package provides ROS2 teleoperation, allowing remote control of motors and actuators via joystick input.

## Features
- Motor and actuator control via GPIO
- Receives joystick commands and translates them to motor actions
- Written in C++

## Dependencies
- ROS2 
- ament_cmake
- rclcpp
- std_msgs
- sensor_msgs
- lgpio

## Building
Clone this package into your ROS2 workspace `src` directory and build with colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select teleop
```

## Usage
Source your workspace and run the teleop node:

```bash
. install/setup.bash
ros2 run teleop motor_controller
```

## Nodes
### motor_controller
- **Subscribed topics:**
  - `/joy` (`sensor_msgs/msg/Joy`): Joystick input
- **Published topics:** None
- **Parameters:**
  - (Add any configurable parameters here)

## Source Structure
- `include/MotorController.h`: Motor controller class
- `src/MotorController.cpp`: Motor controller implementation
- `src/MotorControllerMiddleware.cpp`: ROS2 node for teleoperation

## Maintainer
- emoryducote@gmail.com

## License
See the main LICENSE file in the repository.
