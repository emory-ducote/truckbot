import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory('navigation')
    
    pure_pursuit_config = os.path.join(config_dir, 'config', 'pure_pursuit.yaml')
    path_tracker_config = os.path.join(config_dir, 'config', 'path_tracker.yaml')
    a_star_planner_config = os.path.join(config_dir, 'config', 'a_star_planner.yaml')
    spline_generator_config = os.path.join(config_dir, 'config', 'spline_generator.yaml')
    cost_grid_config = os.path.join(config_dir, 'config', 'cost_grid.yaml')
    
    return LaunchDescription([
        Node(
            package='navigation',
            executable='spline_generator',
            name='spline_generator',
            parameters=[spline_generator_config]
        ),
        Node(
            package='navigation',
            executable='a_star_planner',
            name='a_star_planner',
            parameters=[a_star_planner_config]
        ),
        Node(
            package='navigation',
            executable='path_tracker',
            name='path_tracker',
            parameters=[path_tracker_config]
        ),
        Node(
            package='navigation',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            parameters=[pure_pursuit_config]
        ),
        Node(
            package='navigation',
            executable='cost_grid',
            name='cost_grid',
            parameters=[cost_grid_config]
        )
    ])
