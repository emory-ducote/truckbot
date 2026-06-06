# navigation

## Overview
Path planning and following for truckbot. The package implements a pipeline of four nodes: a static path server, a path tracker, a spline generator, and a pure pursuit controller that outputs velocity commands.

```
simple_path_server ──/global_path──► path_tracker ──/global_path_index──► spline_generator ──/local_path──► pure_pursuit_controller ──/cmd_vel──► control
                                ▲                                                                         ▲
                      localization /local_odom                                              localization /local_odom
```

## Features
- Static path server: loads waypoints from a YAML config and publishes them as a `nav_msgs/Path`
- Nearest-point tracking: finds the closest waypoint on the global path to the vehicle's current pose
- Spline generation: generates a smooth local path ahead of the vehicle using a configurable sample scale
- Pure pursuit control: computes angular velocity commands to follow the local path at a fixed forward speed

## Dependencies
- ROS2
- ament_cmake
- rclcpp
- nav_msgs
- geometry_msgs
- std_msgs

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select navigation
```

## Usage

Launch all four nodes together with the full navigation stack:

```bash
. install/setup.bash
ros2 launch navigation navigation.launch.py
```

Or run individual nodes:

```bash
ros2 run navigation simple_path_server --ros-args --params-file config/simple_path_server.yaml
ros2 run navigation path_tracker
ros2 run navigation spline_generator
ros2 run navigation pure_pursuit_controller
```

## Nodes

### simple_path_server
Publishes a static global path loaded from parameters at a fixed rate.

- **Published topics:**
  - `/global_path` (`nav_msgs/msg/Path`): global waypoint path
- **Parameters:**
  - `path_x` (double[]): x coordinates of waypoints
  - `path_y` (double[]): y coordinates of waypoints
  - `path_frame` (string, default: `map`): frame ID for the path header
  - `path_topic` (string, default: `/global_path`): topic to publish on
  - `publish_rate` (double, default: `5.0`): publish rate in Hz

### path_tracker
Finds the nearest waypoint on the global path to the current vehicle pose and publishes its index.

- **Subscribed topics:**
  - `/local_odom` (`nav_msgs/msg/Odometry`): vehicle pose from localization
  - `/global_path` (`nav_msgs/msg/Path`): global path from `simple_path_server`
- **Published topics:**
  - `/global_path_index` (`std_msgs/msg/Int64`): index of the nearest path waypoint
  - `/nearest_path_point` (`geometry_msgs/msg/PointStamped`): coordinates of the nearest waypoint

### spline_generator
Generates a smooth local path by fitting a spline to the global path ahead of the vehicle.

- **Subscribed topics:**
  - `/global_path` (`nav_msgs/msg/Path`): global waypoint path
  - `/global_path_index` (`std_msgs/msg/Int64`): current vehicle position index in the global path
- **Published topics:**
  - `/local_path` (`nav_msgs/msg/Path`): interpolated local path ahead of the vehicle
- **Parameters:**
  - `sample_scale` (double, default: `4.0`): number of interpolated points per global waypoint interval

### pure_pursuit_controller
Implements a pure pursuit controller that tracks the local path and publishes velocity commands.

- **Subscribed topics:**
  - `/local_odom` (`nav_msgs/msg/Odometry`): vehicle pose from localization
  - `/local_path` (`nav_msgs/msg/Path`): local path from `spline_generator`
- **Published topics:**
  - `/cmd_vel` (`geometry_msgs/msg/Twist`): commanded linear and angular velocity
  - `/global_lookahead` (`geometry_msgs/msg/PointStamped`): lookahead point in map frame (debug)
  - `/local_lookahead` (`geometry_msgs/msg/PointStamped`): lookahead point in vehicle frame (debug)
- **Parameters:**
  - `lookahead_distance` (double, default: `0.5`): pure pursuit lookahead distance in meters

## Source Structure
- `include/PathTracker.h`: nearest-waypoint search
- `include/PurePursuitController.h`: pure pursuit geometry and control computation
- `include/SplineGenerator.h`: spline interpolation
- `include/VehiclePose.h`: 2D pose type (x, y, theta)
- `src/PathTracker.cpp`, `src/PathTrackerMiddleware.cpp`: path tracker implementation and ROS2 node
- `src/PurePursuitController.cpp`, `src/PurePursuitControllerMiddleware.cpp`: pure pursuit implementation and ROS2 node
- `src/SplineGenerator.cpp`, `src/SplineGeneratorMiddleware.cpp`: spline generator implementation and ROS2 node
- `src/SimplePathServerMiddleware.cpp`: static path server ROS2 node
- `config/`: per-node parameter YAML files
- `launch/navigation.launch.py`: launches the full navigation stack

## Maintainer
- emoryducote@gmail.com

## License
MIT
