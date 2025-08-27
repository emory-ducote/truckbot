# control

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
- geometry_msgs
- lgpio

## Building
Clone this package into your ROS2 workspace `src` directory and build with colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select control
```

## Usage
Source your workspace and run the control node:

```bash
. install/setup.bash
ros2 run control motor_controller
```

## Nodes
### motor_controller
- **Subscribed topics:**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): Joystick input
- **Published topics:** None
- **Parameters:**
  - (Add any configurable parameters here)

### teleop
- **Subscribed topics:**
  - `/joy` (`sensor_msgs/msg/Joy`): Joystick input
- **Published topics:**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): commanded velocity output
  - `/cmd_actuator` (`std_msgs/Bool`): commanded scissor lift actuator output
- **Parameters:**
  - (Add any configurable parameters here)

## Source Structure
- `include/MotorController.h`: Motor controller class
- `src/MotorController.cpp`: Motor controller implementation
- `src/MotorControllerMiddleware.cpp`: ROS2 node for receiving control commands
- `src/TeleOp.cpp`: ROS2 node for receiving joystick input and forwarding it

## Maintainer
- emoryducote@gmail.com

## License
See the main LICENSE file in the repository.
