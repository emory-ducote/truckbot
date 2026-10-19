import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dashboard_config = os.path.join(
        get_package_share_directory('dashboard'),
        'config',
        'dashboard.yaml'
    )

    return LaunchDescription([
        Node(
            package='dashboard',
            executable='dashboard',
            name='dashboard',
            parameters=[dashboard_config]
        )
    ])
