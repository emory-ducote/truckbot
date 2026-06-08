import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    control_dir = get_package_share_directory('control')
    localization_dir = get_package_share_directory('localization')
    navigation_dir = get_package_share_directory('navigation')
    perception_dir = get_package_share_directory('perception')
    imu_dir = get_package_share_directory('icm20948_driver')

    return LaunchDescription([

        # --- Static transforms (robot geometry) ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.055', '0', '0.13', '0', '0', '0', 'base_link', 'laser']
        ),

        # --- Subsystems ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_dir, 'launch', 'control.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization_dir, 'launch', 'localization.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_dir, 'launch', 'navigation.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_dir, 'launch', 'voxelization.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(imu_dir, 'launch', 'imu.launch.py')
            )
        ),

    ])
