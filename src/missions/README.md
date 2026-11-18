# missions

## Overview
Mission runner for truckbot. Missions are defined in `config/missions.yaml`, nested as mission, then state, then the actions for that state:

```yaml
PickupAndReturn:
  DriveToPickup:
    Goal: {x: 7.25, y: -1.5}
  RaiseLift:
    LiftEnabled: true
```

Missions run in file order, and so do the states inside each mission. A state can set `Goal` (sent on `/goal_pose`), `LiftEnabled` (sent on `/cmd_actuator`), or both. Every timer tick checks the current state. Once each action it started has reported done (`/goal_reached`, `/lift_done`), the next state is dispatched.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Published |
| `/cmd_actuator` | `std_msgs/Bool` | Published |
| `/goal_reached` | `std_msgs/Bool` | Subscribed (from `pure_pursuit_controller`) |
| `/lift_done` | `std_msgs/Bool` | Subscribed (from `motor_driver`) |
| `/health/heartbeat` | `truckbot_msgs/Heartbeat` | Published |

## Parameters

See `config/mission_runner.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `global_frame` | map | Frame goals are given in |
| `timer_period_ms` | 100 | Mission step period |
| `missions_file` | share/missions/config/missions.yaml | Missions definition file |

## Dependencies
- ROS2
- ament_cmake
- rclcpp
- truckbot_msgs
- geometry_msgs
- std_msgs
- ament_index_cpp
- yaml-cpp
