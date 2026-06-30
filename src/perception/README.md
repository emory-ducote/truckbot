# perception

## Overview
LiDAR-based perception for truckbot. Converts RPLidar scan data into point clouds and processes them for obstacle awareness. Provides three nodes: point cloud downsampling via voxelization, Euclidean cluster-based object detection, and Shi-Tomasi corner detection for landmark features.

## Features
- Converts `sensor_msgs/LaserScan` to PCL point clouds via `laser_geometry`
- Voxel grid downsampling to reduce point cloud density
- Euclidean cluster extraction to detect and localize nearby objects
- Shi-Tomasi / Harris corner detection on a rasterized occupancy image to extract stable landmark features
- Publishes detected features as `visualization_msgs/MarkerArray` for RViz visualization

## Dependencies
- ROS2
- ament_cmake
- rclcpp
- sensor_msgs
- visualization_msgs
- laser_geometry
- pcl_conversions
- PCL (Point Cloud Library)
- OpenCV (`libopencv-dev`, used by the corner detector)

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select perception
```

## Usage

```bash
. install/setup.bash
ros2 run perception voxelization
ros2 run perception feature_detection
ros2 run perception corner_detector
```

Or launch a node with its config:

```bash
ros2 launch perception voxelization.launch.py
ros2 launch perception corner_detector.launch.py
```

## Nodes

### voxelization
Subscribes to the LiDAR scan, projects it to a 3D point cloud, applies voxel grid downsampling, and publishes the resulting points as RViz markers.

- **Subscribed topics:**
  - `/scan` (`sensor_msgs/msg/LaserScan`): raw LiDAR scan from `rplidar_ros`
- **Published topics:**
  - `/cluster_markers` (`visualization_msgs/msg/MarkerArray`): voxelized point cloud as sphere markers
- **Parameters:**
  - `voxel_size` (double, default: `0.05`): voxel leaf size in meters

### feature_detection
Subscribes to the LiDAR scan, projects it to a point cloud, and runs Euclidean cluster extraction. Publishes the centroid of each detected cluster as a marker.

- **Subscribed topics:**
  - `/scan` (`sensor_msgs/msg/LaserScan`): raw LiDAR scan from `rplidar_ros`
- **Published topics:**
  - `/cluster_markers` (`visualization_msgs/msg/MarkerArray`): one marker per detected cluster (centroid position)
- **Parameters:**
  - `cluster_tolerance` (double, default: `0.05`): max distance between points in a cluster (meters)
  - `min_cluster_size` (double, default: `2`): minimum number of points to form a cluster
  - `max_cluster_size` (double, default: `10`): maximum number of points in a cluster

### corner_detector
Rasterizes the LiDAR scan into an occupancy image, runs OpenCV's `goodFeaturesToTrack` (Shi-Tomasi by default, optionally Harris), and maps the detected corners back into the laser frame. Publishes each corner as a marker — these serve as stable landmark features for the localization particle filter. Publishes to `/cluster_markers`, so it is an alternative landmark source to `feature_detection` (run one or the other).

- **Subscribed topics:**
  - `/scan` (`sensor_msgs/msg/LaserScan`): raw LiDAR scan from `rplidar_ros`
- **Published topics:**
  - `/cluster_markers` (`visualization_msgs/msg/MarkerArray`): one marker per detected corner (in the `laser` frame)
- **Parameters:**
  - `map_resolution` (double, default: `0.05`): occupancy image resolution in meters/pixel
  - `map_range` (double, default: `6.0`): half-extent of the occupancy image in meters
  - `max_corners` (int, default: `100`): maximum number of corners to detect
  - `quality_level` (double, default: `0.15`): minimum accepted corner quality (fraction of best corner)
  - `min_distance` (double, default: `0.3`): minimum pixel distance between detected corners
  - `block_size` (int, default: `3`): neighborhood size for corner computation
  - `use_harris_detector` (bool, default: `false`): use Harris instead of Shi-Tomasi
  - `harris_k` (double, default: `0.04`): Harris detector free parameter
  - `wall_thickness` (int, default: `2`): dilation thickness applied to rasterized scan returns
  - `min_corner_distance` (double, default: `0.5`): drop corners closer than this (meters) to the laser origin

## Source Structure
- `include/Voxelization.h`: voxel grid filter wrapper
- `src/Voxelization.cpp`: voxelization implementation
- `src/VoxelizationMiddleware.cpp`: ROS2 node for voxelization
- `include/FeatureDetection.h`: Euclidean cluster extraction wrapper
- `src/FeatureDetection.cpp`: feature detection implementation
- `src/FeatureDetectionMiddleware.cpp`: ROS2 node for feature detection
- `include/CornerDetector.h`: Shi-Tomasi / Harris corner detector wrapper
- `src/CornerDetector.cpp`: corner detection implementation
- `src/CornerDetectorMiddleware.cpp`: ROS2 node for corner detection
- `config/voxelization.yaml`: default parameters for voxelization node
- `config/corner_detector.yaml`: default parameters for corner detector node
- `launch/voxelization.launch.py`: launch file for voxelization node
- `launch/corner_detector.launch.py`: launch file for corner detector node

## Maintainer
- emoryducote@gmail.com

## License
MIT
