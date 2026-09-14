"""统一启动 MoveIt 的 move_group（可选带 RViz），环境由 env 参数选择。

只启动 move_group：不启动 ros2_control / 控制器 / MuJoCo 仿真 / robot_state_publisher。

用法：
    ros2 launch launch_controller start_moveit_group.launch.py env:=space_sim
    ros2 launch launch_controller start_moveit_group.launch.py env:=lab_bench
    ros2 launch launch_controller start_moveit_group.launch.py env:=space_sim launch_rviz:=false

配套控制器（需先启动，否则只能规划不能执行）：
    ros2 launch launch_controller start_traditional_controllers.launch.py \\
        env:=space_sim_mujoco initial_ur_controller:=joint_trajectory_controller
    ros2 launch launch_controller start_traditional_controllers.launch.py env:=lab_bench

环境 -> 配置包（{env}_moveit_config 包内固定结构）：
    config/{env}.srdf                                 # SRDF
    config/kinematics.yaml                            # 运动学
    config/joint_limits.yaml                          # 关节限位
    config/planning_pipelines.yaml                    # 规划管线（OMPL 插件等）
    config/move_group_common.yaml                     # move_group 通用参数
    config/moveit.rviz                                # RViz 配置
    config/moveit_controllers_SimpleControllerManager.yaml
    config/moveit_controllers_Ros2ControlManager.yaml
描述包：{env}_description，几何入口 xacro/{env}.xacro

env 可带 `_mujoco` 后缀（如 space_sim_mujoco）：MoveIt 配置仍复用 {base_env}_moveit_config，
只是 use_sim_time 默认按仿真推断为 true。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ============================================================
# 环境约定：{env}_moveit_config / {env}_description
# ============================================================

#: 支持的基环境（不含 _mujoco 后缀）
SUPPORTED_ENVS = ("lab_bench", "space_sim")

#: 基环境中必须使用仿真时钟的（space_sim 无真实硬件，/joint_states 带仿真时间戳）
SIM_TIME_ENVS = ("space_sim",)

MOVEIT_CONTROLLER_MANAGER_SIMPLE = "moveit_controllers_SimpleControllerManager.yaml"
MOVEIT_CONTROLLER_MANAGER_ROS2 = "moveit_controllers_Ros2ControlManager.yaml"


# ============================================================
# Robot description parameters
# 只保留 MoveIt / RViz 真正需要的几何描述参数
# ============================================================

CAMERA_USE_NOMINAL_EXTRINSICS = "true"

SAFETY_LIMITS = "false"
SAFETY_POS_MARGIN = "0.15"
SAFETY_K_POSITION = "20"
FORCE_ABS_PATHS = "false"


def _split_env(env):
    """space_sim_mujoco -> ("space_sim", True)。"""
    is_mujoco = env.endswith("_mujoco")
    base_env = env[: -len("_mujoco")] if is_mujoco else env
    return base_env, is_mujoco


def _moveit_package(base_env):
    return f"{base_env}_moveit_config"


def _description_package(base_env):
    return f"{base_env}_description"


def _config_path(base_env, *paths):
    return os.path.join(
        get_package_share_directory(_moveit_package(base_env)),
        "config",
        *paths,
    )


def _load_yaml(base_env, *paths):
    with open(_config_path(base_env, *paths), "r") as f:
        return yaml.safe_load(f)


def _load_file(base_env, *paths):
    with open(_config_path(base_env, *paths), "r") as f:
        return f.read()


def _generate_robot_description(base_env):
    """用纯描述包的几何入口生成 robot_description（不含 ros2_control）。"""
    description_file = PathJoinSubstitution(
        [
            FindPackageShare(_description_package(base_env)),
            "xacro",
            f"{base_env}.xacro",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "camera_use_nominal_extrinsics:=",
            CAMERA_USE_NOMINAL_EXTRINSICS,
            " ",
            "safety_limits:=",
            SAFETY_LIMITS,
            " ",
            "safety_pos_margin:=",
            SAFETY_POS_MARGIN,
            " ",
            "safety_k_position:=",
            SAFETY_K_POSITION,
            " ",
            "force_abs_paths:=",
            FORCE_ABS_PATHS,
            " ",
        ]
    )

    return {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }


def _resolve_use_sim_time(context, base_env, is_mujoco):
    """auto/空 -> 按环境推断；显式 true/false 覆盖。"""
    raw = LaunchConfiguration("use_sim_time").perform(context).strip().lower()

    if raw in ("", "auto"):
        return is_mujoco or base_env in SIM_TIME_ENVS
    if raw in ("true", "1", "yes"):
        return True
    if raw in ("false", "0", "no"):
        return False

    raise RuntimeError(
        f"[MoveIt Launch] use_sim_time='{raw}' 非法，可选：auto / true / false。"
    )


def _resolve_use_ros2_control_manager(context):
    raw = LaunchConfiguration("use_ros2_control_manager").perform(context)
    return raw.strip().lower() in ("true", "1", "yes")


def launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration("env").perform(context).strip()
    base_env, is_mujoco = _split_env(env)

    if base_env not in SUPPORTED_ENVS:
        supported = ", ".join(
            list(SUPPORTED_ENVS) + [f"{e}_mujoco" for e in SUPPORTED_ENVS]
        )
        raise RuntimeError(
            f"[MoveIt Launch] 不支持的环境 env='{env}'，可选：{supported}。"
        )

    try:
        get_package_share_directory(_moveit_package(base_env))
    except Exception as exc:  # noqa: BLE001 - 缺包时给一句人话
        raise RuntimeError(
            f"[MoveIt Launch] 找不到配置包 '{_moveit_package(base_env)}'"
            f"（env='{env}'），请先 colcon build：{exc}"
        )

    use_sim_time = _resolve_use_sim_time(context, base_env, is_mujoco)
    use_ros2_control_manager = _resolve_use_ros2_control_manager(context)

    print(
        f"[MoveIt Launch] env={env} -> package={_moveit_package(base_env)}, "
        f"description={_description_package(base_env)}/xacro/{base_env}.xacro, "
        f"use_sim_time={use_sim_time}, "
        f"controller_manager="
        f"{'Ros2ControlManager' if use_ros2_control_manager else 'SimpleControllerManager'}"
    )

    # ============================================================
    # Common MoveIt parameters
    # ============================================================

    robot_description = _generate_robot_description(base_env)

    robot_description_semantic = {
        "robot_description_semantic": _load_file(base_env, f"{base_env}.srdf")
    }

    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(base_env, "kinematics.yaml")
    }

    robot_description_planning = {
        "robot_description_planning": _load_yaml(base_env, "joint_limits.yaml")
    }

    planning_pipelines_yaml = _config_path(base_env, "planning_pipelines.yaml")
    move_group_common_yaml = _config_path(base_env, "move_group_common.yaml")

    moveit_controllers = _load_yaml(
        base_env,
        MOVEIT_CONTROLLER_MANAGER_ROS2
        if use_ros2_control_manager
        else MOVEIT_CONTROLLER_MANAGER_SIMPLE,
    )

    rviz_config = _config_path(base_env, "moveit.rviz")

    # ============================================================
    # move_group（一个节点同时服务 arm_A / arm_B / arm_BOTH）
    # ============================================================

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name=LaunchConfiguration("move_group_node_name"),
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines_yaml,
            move_group_common_yaml,
            moveit_controllers,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ============================================================
    # RViz（可选）
    # ============================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            moveit_controllers,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "env",
                default_value="lab_bench",
                description=(
                    "Environment to launch move_group for "
                    "(lab_bench / space_sim, 可选 _mujoco 后缀)."
                ),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Whether to launch RViz with the MoveIt MotionPlanning plugin.",
            ),
            DeclareLaunchArgument(
                "move_group_node_name",
                default_value="arm_BOTH_move_group",
                description="Name of the move_group node (serves arm_A / arm_B / arm_BOTH).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="auto",
                description=(
                    "auto (默认): space_sim 系列 -> true、lab_bench 系列 -> false；"
                    "也可显式传 true / false。"
                ),
            ),
            DeclareLaunchArgument(
                "use_ros2_control_manager",
                default_value="false",
                description=(
                    "true: 用 moveit_ros_control_interface/Ros2ControlManager；"
                    "false: 用 SimpleControllerManager（双臂并行执行保持 false）。"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
