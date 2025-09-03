import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_controller_config = os.path.join(
        get_package_share_directory('control'),
        'config',
        'motor_controller.yaml'
    )
    return LaunchDescription([
        Node(
            package='control',
            executable='motor_controller',
            name='motor_controller',
            parameters=[motor_controller_config]
        )
    ])
