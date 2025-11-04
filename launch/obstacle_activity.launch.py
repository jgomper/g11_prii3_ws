from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Establecer el modelo de TurtleBot3
        ExecuteProcess(
            cmd=['export', 'TURTLEBOT3_MODEL=waffle'],
            shell=True
        ),
        
        # Lanzar el mundo vacío de Gazebo
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'empty_world.launch.py'],
            output='screen',
            name='gazebo_launch'
        ),
        
        # Esperar a que Gazebo se inicie y luego lanzar los otros nodos
        TimerAction(
            period=10.0,
            actions=[
                # Nodo de evitación de obstáculos
                Node(
                    package='g11_prii3',
                    executable='obstacle_avoider',
                    output='screen',
                    name='obstacle_avoider'
                ),
                
                # Nodo manager de obstáculos (demo automática)
                Node(
                    package='g11_prii3',
                    executable='obstacle_manager',
                    output='screen',
                    name='obstacle_manager'
                ),
                
                # Filtro LIDAR (opcional)
                Node(
                    package='g11_prii3',
                    executable='laser_filter',
                    output='screen',
                    name='laser_filter'
                )
            ]
        )
    ])