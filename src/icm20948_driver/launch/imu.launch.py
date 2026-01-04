import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_config = os.path.join(
        get_package_share_directory('icm20948_driver'),
        'config',
        'imu.yaml'
    )
    return LaunchDescription([
        Node(
            package='icm20948_driver',
            executable='imu',
            name='imu',
            parameters=[imu_config]
        )
    ])
