# dashboard

## Overview
Web dashboard for truckbot health. Subscribes to `/health/system` from the `health` monitor and serves a page of colored tiles, one per module, at `http://<robot-ip>:8080`.

| Color | State |
|-------|-------|
| Green | `GOOD` |
| Yellow | `DEGRADED` |
| Red | `FAILED` |
| Gray | `INIT`, or no `/health/system` data for more than 2 s |

The page polls the `/status` JSON endpoint every 500 ms.

The **Deliver** button posts to `/deliver`, which calls the `/mission/start` service on the mission runner and shows the reply next to the button. The mission runner rejects the call unless system health is `GOOD` and the missions haven't started yet.

## Files

| File | Purpose |
|------|---------|
| `dashboard/dashboard.py` | `Dashboard` class: holds the latest health snapshot and serves the Flask routes (`/`, `/status`, `/deliver`) |
| `dashboard/dashboard_middleware.py` | ROS node subscribing to `/health/system` and feeding `Dashboard`, plus the `/mission/start` client behind `/deliver` |
| `dashboard/templates/index.html` | Page layout |
| `dashboard/static/dashboard.js` | Polls `/status` and renders the tiles |
| `dashboard/static/dashboard.css` | Tile colors and layout |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/health/system` | `truckbot_msgs/SystemHealth` | Subscribed |
| `/mission/start` | `std_srvs/Trigger` | Service client (Deliver button) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | 8080 | Port the dashboard is served on |

## Dependencies
- ROS2
- rclpy
- truckbot_msgs
- std_srvs
- python3-flask (`sudo apt install python3-flask`)
