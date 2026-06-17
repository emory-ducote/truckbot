import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    line_extraction_config = os.path.join(
        get_package_share_directory('perception'),
        'config',
        'line_extraction.yaml'
    )
    return LaunchDescription([
        Node(
            package='perception',
            executable='line_extractor',
            name='line_extraction_middleware',
            output='screen',
            parameters=[line_extraction_config]
        )
    ])
