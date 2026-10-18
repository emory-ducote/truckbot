#!/usr/bin/env bash
# Usage: ./scripts/send_goal.sh [x] [y] [frame_id]
X=${1:-7.5}
Y=${2:--1.0}
FRAME=${3:-map}

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: '$FRAME'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {w: 1.0}}}"
