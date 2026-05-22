from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='spline_generator',
            name='spline_generator_middleware'
        )
    ])
