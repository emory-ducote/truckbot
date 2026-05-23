import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    spline_config = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'spline_generator_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='navigation',
            executable='spline_generator',
            name='spline_generator_middleware',
            parameters=[spline_config]
        )
    ])
