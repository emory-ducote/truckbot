# truckbot_bringup

## Overview
Top-level launch package for truckbot. Assembles all subsystems into a single launch file and defines the robot's static TF geometry.

## TF Tree

```
map
  └── odom          (localization/particle_filter — map→odom correction)
        └── base_link   (localization/ekf — dead-reckoning odom→base_link)
              └── laser (static transform — LIDAR mounting geometry)
```

## Static Transforms

| Parent | Child | Translation (x, y, z) | Yaw |
|--------|-------|------------------------|-----|
| `base_link` | `laser` | 0.055 m, 0 m, 0.13 m | π rad (LIDAR mounted backward) |

## Dependencies
- ROS2
- ament_cmake
- tf2_ros
- control
- localization
- navigation
- perception
- icm20948_driver

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select truckbot_bringup
```

## Usage

Launch the full robot stack:

```bash
. install/setup.bash
ros2 launch truckbot_bringup truckbot.launch.py
```

## Source Structure
- `launch/truckbot.launch.py`: full system launch file

## Maintainer
- emoryducote@gmail.com

## License
MIT
