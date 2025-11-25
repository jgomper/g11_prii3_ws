#!/usr/bin/env python3
"""
autonomous_navigation_demo.launch.py
Launch file para navegación autónoma con detección de ArUcos - Apartado 3.4
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Argumentos para configurar velocidades
    aruco_id_arg = DeclareLaunchArgument(
        'target_aruco_id',
        default_value='0',
        description='ID del ArUco objetivo (0, 1, o 2)'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Velocidad lineal del robot (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed', 
        default_value='0.8',
        description='Velocidad angular del robot (rad/s)'
    )
    
    # 1. Mundo F1L3 en Gazebo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('g11_prii3'),
            '/launch/f1l3_world.launch.py'
        ])
    )
    
    # 2. Detector de ArUcos (inicia inmediatamente)
    aruco_detector_node = Node(
        package='g11_prii3',
        executable='aruco_detector_autonomous',
        name='aruco_detector_autonomous',
        output='screen'
    )
    
    # 3. Navegación autónoma (con delay para que todo se inicialice)
    autonomous_nav_node = TimerAction(
        period=6.0,  # Reducido de 8.0 a 6.0 segundos
        actions=[
            Node(
                package='g11_prii3',
                executable='autonomous_navigation',
                name='autonomous_navigation',
                output='screen',
                parameters=[{
                    'target_aruco_id': LaunchConfiguration('target_aruco_id'),
                    'linear_speed': LaunchConfiguration('linear_speed'),
                    'angular_speed': LaunchConfiguration('angular_speed')
                }]
            )
        ]
    )

    return LaunchDescription([
        aruco_id_arg,
        linear_speed_arg,
        angular_speed_arg,
        world_launch,
        aruco_detector_node,
        autonomous_nav_node
    ])