from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node

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
                Node(
                    package='g11_prii3',
                    executable='draw_obstacle_avoidance',
                    output='screen',
                    name='draw_obstacle_avoidance'
                ),
            ]
        )
    ])