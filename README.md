# truckbot

An autonomous indoor ground vehicle that delivers drinks within an apartment. The robot navigates to users on command, uses a LiDAR for obstacle awareness, and raises a scissor lift to serve drinks at table height.

## Packages

| Package | Description |
|---|---|
| [`truckbot_bringup`](src/truckbot_bringup/) | Top-level launch file and static TF geometry |
| [`control`](src/control/) | Motor control and joystick teleoperation via GPIO |
| [`localization`](src/localization/) | EKF dead-reckoning odometry and a FastSLAM-style particle filter for landmark-based SLAM |
| [`navigation`](src/navigation/) | Path serving, spline generation, and pure pursuit path following |
| [`perception`](src/perception/) | LiDAR point cloud processing, Euclidean clustering, and Shi-Tomasi corner detection |
| [`icm20948_driver`](src/icm20948_driver/) | ROS2 driver for the ICM-20948 IMU (accel/gyro/mag) |
| [`rplidar_ros`](src/rplidar_ros/) | Third-party ROS2 driver for RPLidar |

## Building

```bash
cd ~/ros2_ws
colcon build
```

To build a single package:

```bash
colcon build --packages-select <package_name>
```

## Visualization & Tooling

The [`viz/`](viz/) directory holds offline visualization helpers:

- [`viz/truckbot.rviz`](viz/truckbot.rviz): RViz config for the full robot stack (TF tree, LiDAR, clusters, particles, landmarks)
- [`viz/plot_bag.py`](viz/plot_bag.py): generates a self-contained HTML report (trajectory, pose, velocities, IMU, encoders, LiDAR, and an animated landmark map) from a recorded rosbag2 `.mcap`. Runs without a ROS environment:

  ```bash
  python3 viz/plot_bag.py <bag_dir_or_mcap> -o report.html
  ```

  Requires Python packages `plotly`, `numpy`, and the mcap reader:

  ```bash
  pip install mcap mcap-ros2-support plotly numpy
  ```

## Requirements

- ROS2 Humble
- Eigen3
- lgpio
- PCL (Point Cloud Library)
- OpenCV (perception corner detector)
- fmt

## License

MIT
