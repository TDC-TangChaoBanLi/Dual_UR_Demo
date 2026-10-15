"""只启动 space_sim 的 move_group（默认同时起 RViz）—— 调试用。

    ros2 launch space_sim_moveit_config demo.launch.py
    ros2 launch space_sim_moveit_config demo.launch.py launch_rviz:=false

**只**启动 MoveIt 的 move_group（可选 RViz）：不启动 ros2_control / 控制器 / MuJoCo 仿真 /
robot_state_publisher。仿真与控制器要另外启动：

    ros2 launch launch_controller start_joints_controllers.launch.py \\
        env:=space_sim_mujoco initial_ur_controller:=joint_trajectory_controller

本文件只是 `launch_controller/start_moveit_group.launch.py` 的薄封装（env 写死 space_sim）；
一般场景直接用统一入口即可，两个环境共用同一份实现：

    ros2 launch launch_controller start_moveit_group.launch.py env:=space_sim

环境解析、use_sim_time 推断、规划管线（含 chomp / stomp 参数）全部由统一入口负责，
规划器参数在 space_sim_moveit_config/config/planning_pipelines.yaml 里维护。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CONTROL_PACKAGE = "launch_controller"
MOVEIT_LAUNCH_FILE = "start_moveit_group.launch.py"

#: 本包对应的环境（space_sim 无真实硬件，统一入口会把 use_sim_time 推断为 true）
ENV = "space_sim"


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
            description="auto = 按环境推断（space_sim -> true）；也可显式 true / false。",
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
