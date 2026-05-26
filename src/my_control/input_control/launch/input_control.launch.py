from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='input_control',
            executable='keyboard_input_node',
            name='keyboard_input_node',
            output='screen'
        ),
    ])
