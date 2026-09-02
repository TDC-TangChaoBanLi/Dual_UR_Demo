from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='test_control',
        #     executable='test_ros2_control',
        #     name='test_ros2_control',
        #     output='screen',
        # ),
        Node(
            package='test_control',
            executable='test_move_group',
            name='test_move_group',
            output='screen',
        ),
        # Node(
        #     package='test_control',
        #     executable='test_move_servo',
        #     name='test_move_servo',
        #     output='screen',
        #     parameters=[{
        #         'arm_a_planning_frame': 'world',
        #         'arm_a_tcp_frame': 'arm_A__tcp',
        #         'arm_b_planning_frame': 'world',
        #         'arm_b_tcp_frame': 'arm_B__tcp',
        #         'publish_rate_hz': 100.0,
        #     }],
        # ),
        # Node(
        #     package='test_control',
        #     executable='test_mujoco_control',
        #     name='test_mujoco_control',
        #     output='screen',
        # ),
    ])
