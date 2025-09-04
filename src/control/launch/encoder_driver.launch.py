import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    encoder_config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'encoder_driver.yaml'
    )
    return LaunchDescription([
        Node(
            package='control',
            executable='encoder_driver',
            name='encoder_driver',
            parameters=[encoder_config]
        )
    ])
