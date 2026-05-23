import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    path_tracker_config = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'path_tracker.yaml'
    )
    return LaunchDescription([
        Node(
            package='navigation',
            executable='path_tracker',
            name='path_tracker_middleware',
            parameters=[path_tracker_config]
        )
    ])
