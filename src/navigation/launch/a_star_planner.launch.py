import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    a_star_planner_config = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'a_star_planner.yaml'
    )

    return LaunchDescription([
        Node(
            package='navigation',
            executable='a_star_planner',
            name='a_star_planner_middleware',
            parameters=[a_star_planner_config]
        )
    ])
