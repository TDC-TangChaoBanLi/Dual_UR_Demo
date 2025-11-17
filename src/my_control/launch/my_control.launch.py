from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_control',
            executable='ur_moveit_control',
            name='ur_moveit_control',
            output='screen'
        ),
        # Node(
        #     package='my_control',
        #     executable='my_env_moveit_control',
        #     name='my_env_moveit_control',
        #     output='screen'
        # ),
    ])
