#!/usr/bin/env python3
"""
full_simulation.launch.py
Lanza Gazebo + rover + controladores para el Rover Lunar TMR 2026
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, ExecuteProcess,
                            DeclareLaunchArgument, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lunar_rover_sim')

    # ── Argumentos ──────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── URDF del rover (procesado con xacro) ────────────────────
    urdf_path = os.path.join(pkg, 'urdf', 'rover.urdf.xacro')
    robot_description = Command(['xacro ', urdf_path])

    # ── Gazebo con el mundo lunar ────────────────────────────────
    world_path = os.path.join(pkg, 'worlds', 'lunar_arena.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'false',
            'pause': 'false',
        }.items()
    )

    # ── Robot State Publisher ────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # ── Spawn del rover en Gazebo ────────────────────────────────
    # Posición inicial: zona INICIO (x=1, y=5) mirando hacia FIN (este)
    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rover',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lunar_rover',
            '-x', '1.0',
            '-y', '5.0',
            '-z', '0.12',
            '-Y', '0.0',  # yaw 0 = mirando en +X hacia FIN
        ],
        output='screen'
    )

    # ── Joint State Publisher (para visualización) ───────────────
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── RViz2 ────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg, 'config', 'rover_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Usar tiempo de simulación'),
        gazebo,
        rsp,
        TimerAction(period=2.0, actions=[spawn_rover]),
        TimerAction(period=3.0, actions=[jsp]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
