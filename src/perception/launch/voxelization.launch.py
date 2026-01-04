import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    voxelization_config = os.path.join(
        get_package_share_directory('perception'),
        'config',
        'voxelization.yaml'
    )
    return LaunchDescription([
        Node(
            package='perception',
            executable='voxelization',
            name='voxelization',
            parameters=[voxelization_config]
        )
    ])
