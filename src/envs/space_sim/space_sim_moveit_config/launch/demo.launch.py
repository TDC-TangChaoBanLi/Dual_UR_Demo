"""一键起 MuJoCo 仿真 + MoveIt(move_group) + RViz。

    ros2 launch space_sim_moveit_config demo.launch.py
    ros2 launch space_sim_moveit_config demo.launch.py mujoco_headless:=true launch_rviz:=false

环境名 `space_sim_mujoco` 会被 launch_controller 解析为
`space_sim_mujoco_description`（含 mujoco 插件配置），并复用
`space_sim_description` 的几何与 `space_sim_mujoco_description/config/ros2_control/
traditional_controllers.yaml` 的控制器。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CONTROL_PACKAGE = "launch_controller"
CONTROL_LAUNCH_FILE = "start_traditional_controllers.launch.py"
MOVEIT_LAUNCH_FILE = "start_moveit_group.launch.py"

#: MoveIt 用 MuJoCo 环境唯一可用的轨迹控制器（没有 scaled_joint_trajectory_controller）
INITIAL_UR_CONTROLLER = "joint_trajectory_controller"


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    mujoco_headless = LaunchConfiguration("mujoco_headless")
    mujoco_sim_speed_factor = LaunchConfiguration("mujoco_sim_speed_factor")
    move_group_delay = LaunchConfiguration("move_group_delay")

    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz together with move_group.",
        ),
        DeclareLaunchArgument(
            "mujoco_headless",
            default_value="false",
            description="Run the MuJoCo simulation without its viewer window.",
        ),
        DeclareLaunchArgument(
            "mujoco_sim_speed_factor",
            default_value="1.0",
            description="MuJoCo simulation speed factor (1.0 = real time).",
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
            "env": "space_sim_mujoco",
            "initial_ur_controller": INITIAL_UR_CONTROLLER,
            "launch_rviz": "false",
            "mujoco_headless": mujoco_headless,
            "mujoco_sim_speed_factor": mujoco_sim_speed_factor,
        }.items(),
    )

    moveit_launch = TimerAction(
        period=move_group_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(CONTROL_PACKAGE),
                            "launch",
                            MOVEIT_LAUNCH_FILE,
                        ]
                    )
                ),
                # space_sim_mujoco -> 仍用 space_sim_moveit_config，并自动 use_sim_time=true
                launch_arguments={
                    "env": "space_sim_mujoco",
                    "launch_rviz": launch_rviz,
                }.items(),
            )
        ],
    )

    return LaunchDescription(declared_arguments + [control_launch, moveit_launch])
