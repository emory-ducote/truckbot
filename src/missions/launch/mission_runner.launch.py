import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_runner_config = os.path.join(
        get_package_share_directory('missions'),
        'config',
        'mission_runner.yaml'
    )

    return LaunchDescription([
        Node(
            package='missions',
            executable='mission_runner',
            name='mission_runner',
            parameters=[mission_runner_config]
        )
    ])
