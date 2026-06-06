# truckbot

An autonomous indoor ground vehicle that delivers drinks within an apartment. The robot navigates to users on command, uses a LiDAR for obstacle awareness, and raises a scissor lift to serve drinks at table height.

## Packages

| Package | Description |
|---|---|
| [`control`](src/control/) | Motor control and joystick teleoperation via GPIO |
| [`localization`](src/localization/) | Extended Kalman Filter for IMU sensor fusion and state estimation |
| [`navigation`](src/navigation/) | Path serving, spline generation, and pure pursuit path following |
| [`perception`](src/perception/) | LiDAR point cloud processing and Euclidean clustering |
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

## Requirements

- ROS2 Humble
- Eigen3
- lgpio
- PCL (Point Cloud Library)
- fmt

## License

MIT
