import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    simple_path_config = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'simple_path_server_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='navigation',
            executable='simple_path_server',
            name='simple_path_server_middleware',
            parameters=[simple_path_config]
        )
    ])
