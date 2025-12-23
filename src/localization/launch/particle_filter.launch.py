import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    particle_filter_config = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'particle_filter.yaml'
    )
    return LaunchDescription([
        Node(
            package='localization',
            executable='particle_filter',
            name='particle_filter',
            output='screen',
            parameters=[particle_filter_config]
        )
    ])
