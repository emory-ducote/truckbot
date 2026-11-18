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

The first state is held until `/health/system` reports `GOOD`, so goals aren't sent before navigation is up. A `/goal_reached` report only counts if its coordinates match the current state's goal, which means a stale or repeated report can't skip a state. A state whose goal matches the goal just reached does not resend it.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Published |
| `/cmd_actuator` | `std_msgs/Bool` | Published |
| `/goal_reached` | `geometry_msgs/PointStamped` | Subscribed (from `pure_pursuit_controller`) |
| `/health/system` | `truckbot_msgs/SystemHealth` | Subscribed |
| `/lift_done` | `std_msgs/Bool` | Subscribed (from `motor_driver`) |
| `/mission/status` | `truckbot_msgs/MissionStatus` | Published every tick |
| `/health/heartbeat` | `truckbot_msgs/Heartbeat` | Published |

## Parameters

See `config/mission_runner.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `global_frame` | map | Frame goals are given in |
| `timer_period_ms` | 100 | Mission step period |
| `goal_match_tolerance` | 0.001 | Meters a `/goal_reached` report can be from the current goal and still count |
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
