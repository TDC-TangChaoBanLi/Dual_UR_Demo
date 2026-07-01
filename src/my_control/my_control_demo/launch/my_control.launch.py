from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='my_control_demo',
        #     executable='my_env_ros2_control',
        #     name='my_env_ros2_control',
        #     output='screen',
        # ),
        Node(
            package='my_control_demo',
            executable='my_env_move_group',
            name='my_env_move_group',
            output='screen',
        ),
        # Node(
        #     package='my_control_demo',
        #     executable='my_env_move_servo',
        #     name='my_env_move_servo',
        #     output='screen',
        #     parameters=[{
        #         'servo_target': 'both',
        #         'command_type': 'pose',  # pose, twist, or joint
        #         'duration_sec': 30.0,
        #     }],
        # ),
        # Node(
        #     package='my_control_demo',
        #     executable='my_env_mujoco_control',
        #     name='my_env_mujoco_control',
        #     output='screen',
        # ),
    ])
