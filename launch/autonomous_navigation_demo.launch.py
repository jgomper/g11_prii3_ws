#!/usr/bin/env python3
"""
autonomous_navigation_demo.launch.py
Launch file para navegación autónoma mejorada con Nav2
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Establecer variable de entorno
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'
    
    # Argumentos
    aruco_id_arg = DeclareLaunchArgument(
        'target_aruco_id',
        default_value='5',
        description='ID del ArUco objetivo esperado (5, 6, o 17)'
    )
    
    # 1. Mundo F1L3 en Gazebo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('g11_prii3'),
            '/launch/f1l3_world.launch.py'
        ])
    )
    
    # 2. Nav2 Bringup - ¡ESTO ES LO QUE FALTABA!
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': os.path.join(get_package_share_directory('g11_prii3'), 'worlds', 'mapa_f1l3.yaml'),
            'params_file': os.path.join(get_package_share_directory('g11_prii3'), 'g11_prii3_nav_turtlebot', 'nav2_params.yaml'),
            'use_sim_time': 'True',
            'autostart': 'True'
        }.items()
    )
    
    # 3. Navegación autónoma mejorada (con delay para que Nav2 se inicialice)
    autonomous_nav_node = TimerAction(
        period=10.0,  # Esperar 10 segundos para que Nav2 esté listo
        actions=[
            Node(
                package='g11_prii3',
                executable='autonomous_navigation',
                name='autonomous_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True
                }]
            )
        ]
    )

    return LaunchDescription([
        aruco_id_arg,
        world_launch,
        nav2_bringup_launch,
        autonomous_nav_node
    ])
