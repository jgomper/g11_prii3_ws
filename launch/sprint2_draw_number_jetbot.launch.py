from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    draw_number_node = Node(
        package='g11_prii3',
        executable='draw_number_jetbot',
        name='draw_number_jetbot',
        output='screen'
    )

    return LaunchDescription([
        draw_number_node
    ])

