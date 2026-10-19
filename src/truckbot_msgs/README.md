# truckbot_msgs

## Overview
Custom message definitions shared across truckbot packages.

## Messages

### `Heartbeat`
Health heartbeat published by each module at 1 Hz on `/health/heartbeat`.

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | `builtin_interfaces/Time` | Time the heartbeat was published |
| `node_name` | `string` | Name of the publishing node |
| `state` | `uint8` | One of the state constants below |
| `detail` | `string` | Short human-readable reason for the current state |

| State | Value | Meaning |
|-------|-------|---------|
| `INIT` | 0 | Node is starting up |
| `GOOD` | 1 | Node is operating normally |
| `FAILED` | 2 | Node has hit an error |

## Dependencies
- ROS2
- ament_cmake
- rosidl_default_generators
- builtin_interfaces
