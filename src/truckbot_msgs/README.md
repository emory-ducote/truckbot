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
| `DEGRADED` | 2 | Node is running but could not do its job (e.g. no path to goal) |
| `FAILED` | 3 | Node has hit an error |

### `SystemHealth`
Aggregated health published by the health monitor on `/health/system`.

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | `builtin_interfaces/Time` | Time the monitor evaluated health |
| `state` | `uint8` | Worst state across all modules, using the `Heartbeat` constants |
| `modules` | `Heartbeat[]` | Effective state of each module, including timeouts applied by the monitor |

## Dependencies
- ROS2
- ament_cmake
- rosidl_default_generators
- builtin_interfaces
