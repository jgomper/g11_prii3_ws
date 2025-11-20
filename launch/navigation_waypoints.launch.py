from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Obtener el directorio home del usuario
    home_dir = os.path.expanduser('~')
    map_path = os.path.join(home_dir, 'map.yaml')
    
    return LaunchDescription([
        # Argumentos de lanzamiento
        DeclareLaunchArgument(
            'model',
            default_value='burger',
            description='TurtleBot3 model type'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'run_waypoints',
            default_value='true',
            description='Run waypoint navigation automatically'
        ),
        
        # Lanzar Gazebo con el mundo TurtleBot3
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py',
                'use_sim_time:=', LaunchConfiguration('use_sim_time')
            ],
            output='screen'
        ),
        
        # Esperar a que Gazebo se inicie
        TimerAction(
            period=8.0,
            actions=[
                # Lanzar navegación con el mapa guardado
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                        'use_sim_time:=', LaunchConfiguration('use_sim_time'),
                        'map:=', map_path
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Esperar a que la navegación se inicie
        TimerAction(
            period=15.0,
            actions=[
                # Lanzar el nodo de waypoints
                Node(
                    package='g11_prii3',
                    executable='waypoint_navigator',
                    name='waypoint_navigator',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time')
                    }],
                    condition=IfCondition(LaunchConfiguration('run_waypoints'))
                )
            ]
        )
    ])