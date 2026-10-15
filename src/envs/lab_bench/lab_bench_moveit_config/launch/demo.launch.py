"""只启动 lab_bench 的 move_group（默认同时起 RViz）—— 调试用。

    ros2 launch lab_bench_moveit_config demo.launch.py
    ros2 launch lab_bench_moveit_config demo.launch.py launch_rviz:=false

**只**启动 MoveIt 的 move_group（可选 RViz）：不启动 ros2_control / 控制器 / robot_state_publisher。
真机 / 假硬件控制器要另外启动：

    ros2 launch launch_controller start_joints_controllers.launch.py \\
        env:=lab_bench use_fake_hardware:=true

本文件只是 `launch_controller/start_moveit_group.launch.py` 的薄封装（env 写死 lab_bench）；
一般场景直接用统一入口即可（两个环境共用同一份实现）：

    ros2 launch launch_controller start_moveit_group.launch.py env:=lab_bench

环境解析、use_sim_time 推断（lab_bench 有真实硬件参数文件 -> false）、规划管线全部由统一入口
负责；规划器参数在 lab_bench_moveit_config/config/planning_pipelines.yaml 里维护。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CONTROL_PACKAGE = "launch_controller"
MOVEIT_LAUNCH_FILE = "start_moveit_group.launch.py"

#: 本包对应的环境（lab_bench 是真机环境，统一入口会把 use_sim_time 推断为 false）
ENV = "lab_bench"


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the MoveIt MotionPlanning plugin.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="auto",
            description="auto = 按环境推断（lab_bench -> false）；也可显式 true / false。",
        ),
        DeclareLaunchArgument(
            "move_group_node_name",
            default_value="arm_BOTH_move_group",
            description="Name of the move_group node (serves arm_A / arm_B / arm_BOTH).",
        ),
        DeclareLaunchArgument(
            "use_ros2_control_manager",
            default_value="false",
            description=(
                "true: moveit_ros_control_interface/Ros2ControlManager；"
                "false: SimpleControllerManager（双臂并行执行保持 false）。"
            ),
        ),
    ]

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(CONTROL_PACKAGE), "launch", MOVEIT_LAUNCH_FILE]
            )
        ),
        launch_arguments={
            "env": ENV,
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "move_group_node_name": LaunchConfiguration("move_group_node_name"),
            "use_ros2_control_manager": LaunchConfiguration("use_ros2_control_manager"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [move_group_launch])