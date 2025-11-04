from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Establecer el modelo de TurtleBot3
    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )
    
    # Obtener el directorio de turtlebot3_gazebo
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Usar el launch COMPLETO de turtlebot3_gazebo que incluye TODO
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_turtlebot3_gazebo,
            '/launch/turtlebot3_world.launch.py'
        ])
    )
    
    # Tu nodo de dibujo con delay
    draw_collision_avoidance_node = ExecuteProcess(
        cmd=[
            'bash', '-c', 
            'sleep 20 && ros2 run g11_prii3 draw_collision_avoidance'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        set_turtlebot3_model,
        turtlebot3_gazebo_launch,
        draw_collision_avoidance_node,
    ])