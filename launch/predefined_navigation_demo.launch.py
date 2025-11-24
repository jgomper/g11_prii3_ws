#!/usr/bin/env python3
"""
predefined_navigation_demo.launch.py
Launch file simplificado para navegacion predefinida SOLO en simulacion
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
        period=5.0,
        actions=[
            Node(
                package='g11_prii3',  # CAMBIADO: Usar el paquete principal
                executable='predefined_navigation',  # El ejecutable está en g11_prii3
                name='predefined_navigation',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_launch,
        predefined_nav_node
    ])