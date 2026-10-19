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

## Files

| File | Purpose |
|------|---------|
| `dashboard/dashboard.py` | `Dashboard` class: holds the latest health snapshot and serves the Flask routes (`/`, `/status`) |
| `dashboard/dashboard_middleware.py` | ROS node subscribing to `/health/system` and feeding `Dashboard` |
| `dashboard/templates/index.html` | Page layout |
| `dashboard/static/dashboard.js` | Polls `/status` and renders the tiles |
| `dashboard/static/dashboard.css` | Tile colors and layout |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/health/system` | `truckbot_msgs/SystemHealth` | Subscribed |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | 8080 | Port the dashboard is served on |

## Dependencies
- ROS2
- rclpy
- truckbot_msgs
- python3-flask (`sudo apt install python3-flask`)
