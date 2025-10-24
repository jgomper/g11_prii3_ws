from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Establecer el modelo de TurtleBot3
        ExecuteProcess(
            cmd=['export', 'TURTLEBOT3_MODEL=burger'],
            shell=True
        ),
        
        # Lanzar el mundo vacío de Gazebo
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'empty_world.launch.py'],
            output='screen'
        ),
        
        # Opcional: Lanzar el nodo de teleoperación
        Node(
            package='g11_prii3',
            executable='turtlebot3_teleop_keyboard',
            output='screen',
            name='teleop_keyboard'
        )
    ])
