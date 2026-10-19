import os
import subprocess
import sys
import yaml
import math
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_bag_report(bag_dir, report_script):
    """Render the Plotly report for a finished bag, detached from launch.

    Launch refuses to start new ExecuteProcess actions once it is shutting
    down (Ctrl+C), so the report is spawned directly in its own session; it
    keeps running after launch exits and isn't caught by launch's SIGINT.
    """
    def _spawn(context):
        if not report_script or not os.path.isdir(bag_dir):
            return
        name = os.path.basename(bag_dir)
        log = open(os.path.join(bag_dir, name + '_report.log'), 'w')
        subprocess.Popen(
            [sys.executable, report_script, bag_dir, '--no-show',
             '-o', os.path.join(bag_dir, name + '_report.html')],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f'[bag report] generating {name}_report.html in {bag_dir} '
              f'(log: {name}_report.log)')
    return OpaqueFunction(function=_spawn)


def generate_launch_description():
    control_dir = get_package_share_directory('control')
    localization_dir = get_package_share_directory('localization')
    navigation_dir = get_package_share_directory('navigation')
    perception_dir = get_package_share_directory('perception')
    imu_dir = get_package_share_directory('icm20948_driver')
    health_dir = get_package_share_directory('health')
    dashboard_dir = get_package_share_directory('dashboard')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    bringup_dir = get_package_share_directory('truckbot_bringup')

    with open(os.path.join(bringup_dir, 'config', 'recorder.yaml')) as f:
        recorder_cfg = yaml.safe_load(f)['recorder']
    record_dir = os.path.expanduser(recorder_cfg['record_dir'])
    report_script = os.path.expanduser(recorder_cfg.get('report_script') or '')
    bag_dir = os.path.join(record_dir, 'truckbot_' + datetime.now().strftime('%Y%m%d_%H%M%S'))

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_dir],
        output='screen'
    )

    return LaunchDescription([

        # --- Static transforms (robot geometry) ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.055', '0', '0.13', str(math.pi), '0', '0', 'base_link', 'laser']
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
                os.path.join(perception_dir, 'launch', 'corner_detector.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(imu_dir, 'launch', 'imu.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization_dir, 'launch', 'occupancy_grid.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(health_dir, 'launch', 'health_monitor.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dashboard_dir, 'launch', 'dashboard.launch.py')
            )
        ),

        # --- Data recording ---
        bag_record,
        RegisterEventHandler(OnProcessExit(
            target_action=bag_record,
            on_exit=[generate_bag_report(bag_dir, report_script)],
        )),

    ])
