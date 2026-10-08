"""Start the reusable STOMP task server and optional YAML flow client."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    flow = LaunchConfiguration("flow")
    return LaunchDescription([
        DeclareLaunchArgument("run_flow", default_value="true"),
        DeclareLaunchArgument(
            "flow",
            default_value=PathJoinSubstitution([
                FindPackageShare("space_sim_task_control"), "config", "assembly_flow.yaml"
            ]),
        ),
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("task_log_level", default_value="info",
                             description="Task process log level; debug includes motion poses and joint states"),
        Node(
            package="space_sim_task_control",
            executable="space_sim_task_server",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("task_log_level")],
            parameters=[{"use_sim_time": True, "seed": LaunchConfiguration("seed")}],
        ),
        Node(
            package="space_sim_task_control",
            executable="assembly_flow.py",
            output="screen",
            arguments=["--flow", flow],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("run_flow")),
        ),
    ])
