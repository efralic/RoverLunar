#!/usr/bin/env python3
"""
navigation.launch.py
Lanza SLAM Toolbox + Nav2 para navegación autónoma del Rover Lunar
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lunar_rover_sim')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    slam_pkg = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    # ── SLAM Toolbox ─────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items()
    )

    # ── Nav2 ─────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items()
    )

    # ── robot_localization (EKF: odom + IMU) ─────────────────────
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ekf,
        slam,
        nav2,
    ])
