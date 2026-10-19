import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    health_monitor_config = os.path.join(
        get_package_share_directory('health'),
        'config',
        'health_monitor.yaml'
    )

    return LaunchDescription([
        Node(
            package='health',
            executable='health_monitor',
            name='health_monitor',
            parameters=[health_monitor_config]
        )
    ])
