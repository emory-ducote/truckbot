import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    corner_detector_config = os.path.join(
        get_package_share_directory('perception'),
        'config',
        'corner_detector.yaml'
    )
    return LaunchDescription([
        Node(
            package='perception',
            executable='corner_detector',
            name='corner_detector_middleware',
            output='screen',
            parameters=[corner_detector_config]
        )
    ])
