import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory('navigation')
    
    pure_pursuit_config = os.path.join(config_dir, 'config', 'pure_pursuit.yaml')
    path_tracker_config = os.path.join(config_dir, 'config', 'path_tracker.yaml')
    simple_path_server_config = os.path.join(config_dir, 'config', 'simple_path_server.yaml')
    spline_generator_config = os.path.join(config_dir, 'config', 'spline_generator.yaml')
    
    return LaunchDescription([
        Node(
            package='navigation',
            executable='spline_generator',
            name='spline_generator_middleware',
            parameters=[spline_generator_config]
        ),
        Node(
            package='navigation',
            executable='simple_path_server',
            name='simple_path_server_middleware',
            parameters=[simple_path_server_config]
        ),
        Node(
            package='navigation',
            executable='path_tracker',
            name='path_tracker_middleware',
            parameters=[path_tracker_config]
        ),
        Node(
            package='navigation',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller_middleware',
            parameters=[pure_pursuit_config]
        )
    ])
