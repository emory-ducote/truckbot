# localization

## Overview
State estimation for truckbot using two complementary approaches: an Extended Kalman Filter (EKF) for dead-reckoning odometry, and a particle filter for landmark-based SLAM using LiDAR cluster detections.

## Features
- EKF dead-reckoning from wheel encoder odometry, published as `nav_msgs/Odometry` with covariance
- FastSLAM-style particle filter: builds and maintains a landmark map from LiDAR cluster detections while estimating robot pose
- Particle filter publishes all particle poses and the best-weight pose estimate for visualization in RViz
- Broadcasts a `map → laser` TF transform from the highest-weight particle

## Dependencies
- ROS2
- ament_cmake
- rclcpp
- nav_msgs
- geometry_msgs
- sensor_msgs
- visualization_msgs
- tf2_ros
- Eigen3
- fmt

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select localization
```

## Usage

Run the EKF node:

```bash
. install/setup.bash
ros2 run localization ekf
```

Launch the particle filter with its config:

```bash
ros2 launch localization particle_filter.launch.py
```

## Nodes

### ekf
Dead-reckoning odometry filter. Integrates wheel encoder velocity estimates over time to produce a `nav_msgs/Odometry` pose estimate with covariance.

- **Subscribed topics:**
  - `/odom` (`geometry_msgs/msg/Twist`): wheel encoder velocity from `control/encoder_driver`
- **Published topics:**
  - `/local_odom` (`nav_msgs/msg/Odometry`): integrated pose estimate with 3×3 EKF covariance (x, y, yaw)
- **Parameters:** None (EKF gains are set at compile time)

### particle_filter
Landmark-based SLAM using a FastSLAM-style particle filter. Each particle maintains its own landmark map, updated from LiDAR cluster detections. Motion is driven by encoder odometry. The highest-weight particle's pose is published as the best estimate.

- **Subscribed topics:**
  - `/cluster_markers` (`visualization_msgs/msg/MarkerArray`): detected object clusters from `perception/feature_detection`
  - `/odom` (`geometry_msgs/msg/Twist`): wheel encoder velocity from `control/encoder_driver`
- **Published topics:**
  - `/heaviest_particle_pose` (`geometry_msgs/msg/PoseStamped`): pose of the highest-weight particle
  - `/all_particles_poses` (`geometry_msgs/msg/PoseArray`): poses of all particles (for RViz visualization)
  - `/heaviest_particle_landmarks` (`visualization_msgs/msg/MarkerArray`): landmark map of the highest-weight particle
- **TF broadcasts:**
  - `map → laser`: transform derived from the highest-weight particle's pose
- **Parameters:**
  - `num_particles` (int, default: `15`): number of particles
  - `max_range` (double, default: `6.0`): maximum sensor range in meters
  - `max_angle` (double, default: `250.0`): maximum sensor field of view in degrees
  - `measurement_noise_range` (double, default: `0.05`): range measurement noise in meters
  - `measurement_noise_bearing` (double, default: `0.01`): bearing measurement noise in rad
  - `neff_threshold` (double, default: `0.3`): effective particle ratio threshold for resampling
  - `new_particle_threshold` (double, default: `0.01`): weight threshold below which new particles are added
  - `new_particle_increase` (int, default: `1`): number of new particles added per low-weight event
  - `linear_velocity_alpha_1/2` (double): motion noise parameters for linear velocity
  - `angular_velocity_alpha_1/2` (double): motion noise parameters for angular velocity

## Source Structure
- `include/EKF.h`: EKF class definition
- `src/EKF.cpp`: EKF implementation
- `src/EKFMiddleware.cpp`: ROS2 node for EKF
- `include/ParticleFilter.h`: particle filter class definition
- `include/Particle.h`: particle type (pose + landmark map + weight)
- `include/Landmark.h`: landmark type
- `include/LocalizationHelpers.h`: shared math utilities
- `include/PersistentKDTree.h`: KD-tree for efficient landmark association
- `src/ParticleFilter.cpp`: particle filter implementation
- `src/ParticleFilterMiddleware.cpp`: ROS2 node for particle filter
- `src/ParticleFilterSimulator.cpp`: offline simulator for testing the filter
- `config/particle_filter.yaml`: particle filter parameters
- `launch/particle_filter.launch.py`: particle filter launch file

## Maintainer
- emoryducote@gmail.com

## License
MIT
