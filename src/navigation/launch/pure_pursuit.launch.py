import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pure_pursuit_config = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'pure_pursuit.yaml'
    )
    return LaunchDescription([
        Node(
            package='navigation',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller_middleware',
            parameters=[pure_pursuit_config]
        )
    ])
