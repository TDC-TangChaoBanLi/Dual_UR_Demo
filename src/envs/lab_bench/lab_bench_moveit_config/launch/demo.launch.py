"""一键起 lab_bench 控制器 + MoveIt(move_group) + RViz。

    ros2 launch lab_bench_moveit_config demo.launch.py
    ros2 launch lab_bench_moveit_config demo.launch.py use_fake_hardware:=true launch_rviz:=false

与 `space_sim_moveit_config/demo.launch.py` 结构对齐：先起 launch_controller 的控制器，
再起 move_group（统一由 `launch_controller/launch/start_moveit_group.launch.py` 提供）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CONTROL_PACKAGE = "launch_controller"
CONTROL_LAUNCH_FILE = "start_traditional_controllers.launch.py"
MOVEIT_LAUNCH_FILE = "start_moveit_group.launch.py"

#: 本 demo 固定启动 lab_bench 环境（真机默认 scaled_joint_trajectory_controller）
ENV = "lab_bench"


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    ur_headless_mode = LaunchConfiguration("ur_headless_mode")
    move_group_delay = LaunchConfiguration("move_group_delay")

    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz together with move_group.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use mock hardware for UR and Robotiq (no real robot attached).",
        ),
        DeclareLaunchArgument(
            "ur_headless_mode",
            default_value="false",
            description="Enable UR headless mode.",
        ),
        DeclareLaunchArgument(
            "move_group_delay",
            default_value="3.0",
            description="Seconds to wait after the controllers before starting move_group.",
        ),
    ]

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(CONTROL_PACKAGE), "launch", CONTROL_LAUNCH_FILE]
            )
        ),
        launch_arguments={
            "env": ENV,
            "use_fake_hardware": use_fake_hardware,
            "ur_headless_mode": ur_headless_mode,
            "launch_rviz": "false",
        }.items(),
    )

    moveit_launch = TimerAction(
        period=move_group_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(CONTROL_PACKAGE), "launch", MOVEIT_LAUNCH_FILE]
                    )
                ),
                launch_arguments={
                    "env": ENV,
                    "launch_rviz": launch_rviz,
                }.items(),
            )
        ],
    )

    return LaunchDescription(declared_arguments + [control_launch, moveit_launch])