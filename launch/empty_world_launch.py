from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    # Establecer el modelo de TurtleBot3
    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )
    
    # Mundo vacío de Gazebo
    world_path = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'worlds',
        'empty.world'
    )
    
    # Launch de Gazebo server con mundo vacío
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gzserver.launch.py'
        ]),
        launch_arguments={'world': world_path}.items()
    )
    
    # Launch de Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gzclient.launch.py'
        ])
    )
    
    # Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot3_gazebo'),
            '/launch/robot_state_publisher.launch.py'
        ])
    )
    
    # Spawn del TurtleBot3
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Tu nodo de dibujo del número 11
    draw_number_node = Node(
        package='g11_prii3',
        executable='draw_number_simulation',
        name='draw_number_simulation',
        output='screen'
    )
    
    return LaunchDescription([
        set_turtlebot3_model,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        draw_number_node,
    ])