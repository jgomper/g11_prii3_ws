from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value='waffle'
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'empty_world.launch.py'],
            output='screen',
            name='gazebo_launch'
        ),
        
        TimerAction(
            period=5.0,
            actions=[
                # Controlador de trayectoria completo
                Node(
                    package='g11_prii3',
                    executable='trajectory_controller',
                    output='screen',
                    name='trajectory_controller'
                ),
            ]
        )
    ])