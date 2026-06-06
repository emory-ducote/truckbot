# control

## Overview
Motor control for truckbot. Drives four skid-steer wheels and a scissor lift via GPIO (lgpio), with PID speed control using quadrature encoder feedback. Includes a teleoperation node for joystick-driven control.

## Features
- Skid-steer differential drive for 4 wheel motors
- Scissor lift control (raise/lower)
- Per-wheel PID speed control with encoder feedback
- Quadrature encoder odometry (publishes linear and angular velocity)
- Joystick teleoperation via `sensor_msgs/Joy`

## Dependencies
- ROS2
- ament_cmake
- rclcpp
- std_msgs
- sensor_msgs
- geometry_msgs
- lgpio

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select control
```

## Usage

Launch the motor driver and encoder driver together:

```bash
. install/setup.bash
ros2 launch control control.launch.py
```

Run the teleoperation node separately (requires a joystick connected):

```bash
ros2 run control teleop
```

## Nodes

### motor_driver
Receives velocity and actuator commands and drives the motors via GPIO PWM. Runs a PID control loop at 100 Hz per wheel using encoder speed feedback.

- **Subscribed topics:**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): target linear and angular velocity
  - `/cmd_actuator` (`std_msgs/msg/Bool`): `true` raises the lift, `false` lowers it
  - `/encoder/left_front`, `/encoder/left_rear`, `/encoder/right_front`, `/encoder/right_rear` (`std_msgs/msg/Float64`): measured wheel speeds from `encoder_driver`
- **Published topics:** None
- **Parameters:**
  - `vehicle_width` (double, default: `0.2`): track width in meters, used for skid-steer kinematics
  - `wheel_radius` (double, default: `0.05`): wheel radius in meters
  - `max_wheel_motor_rpm` (int, default: `251`): motor speed cap
  - `Kp`, `Ki`, `Kd` (double, defaults: `2.0`, `1.5`, `0.01`): PID gains for wheel speed control
  - `deadband` (double, default: `0.02`): minimum command magnitude before output is applied
  - `wheel_control_topic` (string, default: `/cmd_vel`)
  - `actuator_control_topic` (string, default: `/cmd_actuator`)
  - GPIO pin parameters: `chip`, `right_front_one/two`, `right_rear_one/two`, `left_front_one/two`, `left_rear_one/two`, `lift_one/two`

### encoder_driver
Reads quadrature encoders from all four wheels via GPIO interrupts and publishes per-wheel speeds and a body-frame odometry twist.

- **Subscribed topics:** None (reads GPIO directly)
- **Published topics:**
  - `/encoder/left_front`, `/encoder/left_rear`, `/encoder/right_front`, `/encoder/right_rear` (`std_msgs/msg/Float64`): individual wheel speeds in m/s
  - `/odom` (`geometry_msgs/msg/Twist`): averaged linear velocity and angular velocity derived from wheel speed differential
- **Parameters:**
  - `wheel_radius` (double, default: `0.05`): wheel radius in meters
  - `wheel_separation` (double, default: `0.2`): track width in meters
  - `encoder_cpr` (int, default: `700`): encoder counts per revolution
  - `encoder_ticks_per_revolution` (int, default: `4`): quadrature ticks per count
  - `update_rate` (int, default: `50`): publish rate in Hz
  - GPIO pin parameters: `chip`, `right_front_a/b`, `right_rear_a/b`, `left_front_a/b`, `left_rear_a/b`

### teleop
Translates joystick input into velocity and lift commands.

- **Subscribed topics:**
  - `/joy` (`sensor_msgs/msg/Joy`): joystick input
- **Published topics:**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): linear.x from left stick Y (×0.75), angular.z from right stick X (×5.0)
  - `/cmd_actuator` (`std_msgs/msg/Bool`): Y button raises the lift, A button lowers it

## Source Structure
- `include/MotorDriver.h`: per-motor PID controller and GPIO PWM driver
- `src/MotorDriver.cpp`: motor driver implementation
- `src/MotorDriverMiddleware.cpp`: ROS2 node for motor control
- `include/EncoderDriver.h`: quadrature encoder reader
- `src/EncoderDriver.cpp`: encoder driver implementation
- `src/EncoderDriverMiddleware.cpp`: ROS2 node for encoder odometry
- `src/TeleOpMiddleware.cpp`: ROS2 node for joystick teleoperation
- `config/motor_driver.yaml`: motor driver parameters and GPIO pin assignments
- `config/encoder_driver.yaml`: encoder driver parameters and GPIO pin assignments
- `launch/control.launch.py`: launches motor driver and encoder driver together

## Maintainer
- emoryducote@gmail.com

## License
MIT
