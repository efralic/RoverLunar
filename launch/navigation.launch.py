#!/usr/bin/env python3
"""
navigation.launch.py
Lanza Nav2 con mapa pre-cargado + AMCL para localizacion
(reemplaza SLAM Toolbox con localizacion en mapa estatico)
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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg, 'maps', 'arena_lunar.yaml')

    # Localizacion: map_server + AMCL
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_pkg, 'launch', 'localization_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': nav2_params,
        }.items()
    )

    # Nav2 (controller, planner, behaviors, bt_navigator, etc)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items()
    )

    # EKF (fusiona odom + IMU)
    #ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    #ekf = Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    #    remappings=[('odometry/filtered', 'odom')]
    #)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
     #   ekf,
        localization,
        nav2,
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            arguments=['/cmd_vel', '/rover/cmd_vel'],
            output='screen'
        ),
    ])
