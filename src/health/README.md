# health

## Overview
System health monitor for truckbot. Every module publishes a `truckbot_msgs/Heartbeat` at 1 Hz on `/health/heartbeat`; the monitor keeps the last heartbeat from each node and publishes the combined result on `/health/system` at 10 Hz.

## Module States

Each module's reported state is passed through, except a module that has not reported within `heartbeat_timeout` is `FAILED` with detail `timed out`. Modules appear once they send their first heartbeat.

The overall system state is the worst module state: `FAILED`, then `DEGRADED`, then `INIT`, then `GOOD`. With no modules reporting it is `INIT`.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/health/heartbeat` | `truckbot_msgs/Heartbeat` | Subscribed |
| `/health/system` | `truckbot_msgs/SystemHealth` | Published |

## Parameters

See `config/health_monitor.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `heartbeat_timeout` | 3.0 | Seconds without a heartbeat before a module is `FAILED` |

## Dependencies
- ROS2
- ament_cmake
- rclcpp
- truckbot_msgs
