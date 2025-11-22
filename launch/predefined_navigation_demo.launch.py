#!/usr/bin/env python3
"""
predefined_navigation_demo.launch.py
Launch file para Gazebo + Navegacion predefinida SOLO
PBI 3.3 - Sprint 3
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. Solo el mundo F1L3 en Gazebo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('g11_prii3'),
            '/launch/f1l3_world.launch.py'
        ])
    )
    
    # 2. Nodo de navegacion predefinida (con delay para que Gazebo se inicialice)
    predefined_nav_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='g11_prii3_nav_turtlebot',
                executable='predefined_navigation',
                name='predefined_navigation',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        world_launch,
        predefined_nav_node
    ])