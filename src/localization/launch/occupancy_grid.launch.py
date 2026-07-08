import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    occupancy_grid_config = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'occupancy_grid.yaml'
    )
    return LaunchDescription([
        Node(
            package='localization',
            executable='occupancy_grid',
            name='occupancy_grid',
            output='screen',
            parameters=[occupancy_grid_config]
        )
    ])
