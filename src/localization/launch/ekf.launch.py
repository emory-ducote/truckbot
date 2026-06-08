from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localization',
            executable='ekf',
            name='ekf',
            output='screen',
        ),
    ])
