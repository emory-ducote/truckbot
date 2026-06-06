# perception

## Overview
LiDAR-based perception for truckbot. Converts RPLidar scan data into point clouds and processes them for obstacle awareness. Provides two nodes: one for point cloud downsampling via voxelization and one for Euclidean cluster-based object detection.

## Features
- Converts `sensor_msgs/LaserScan` to PCL point clouds via `laser_geometry`
- Voxel grid downsampling to reduce point cloud density
- Euclidean cluster extraction to detect and localize nearby objects
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
```

Or launch voxelization with its config:

```bash
ros2 launch perception voxelization.launch.py
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

## Source Structure
- `include/Voxelization.h`: voxel grid filter wrapper
- `src/Voxelization.cpp`: voxelization implementation
- `src/VoxelizationMiddleware.cpp`: ROS2 node for voxelization
- `include/FeatureDetection.h`: Euclidean cluster extraction wrapper
- `src/FeatureDetection.cpp`: feature detection implementation
- `src/FeatureDetectionMiddleware.cpp`: ROS2 node for feature detection
- `config/voxelization.yaml`: default parameters for voxelization node
- `launch/voxelization.launch.py`: launch file for voxelization node

## Maintainer
- emoryducote@gmail.com

## License
MIT
