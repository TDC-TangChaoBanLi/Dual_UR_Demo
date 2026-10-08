"""统一启动 MoveIt 的 move_group（可选带 RViz），环境由 env 参数选择。

只启动 move_group：不启动 ros2_control / 控制器 / MuJoCo 仿真 / robot_state_publisher。

用法：
    ros2 launch launch_controller start_moveit_group.launch.py env:=space_sim
    ros2 launch launch_controller start_moveit_group.launch.py env:=lab_bench
    ros2 launch launch_controller start_moveit_group.launch.py env:=space_sim launch_rviz:=false

env 不是固定枚举：只要是任意已构建的 `{env}_moveit_config` 就能起（见下面的“环境解析”）。

配套控制器（需先启动，否则只能规划不能执行）：
    ros2 launch launch_controller start_traditional_controllers.launch.py \\
        env:=space_sim_mujoco initial_ur_controller:=joint_trajectory_controller
    ros2 launch launch_controller start_traditional_controllers.launch.py env:=lab_bench

环境解析（**没有白名单**，直接按包名找 —— 新增环境只需建包，不用改这个文件）：
    env=<name>  -> MoveIt 配置包 {name}_moveit_config，描述包 {name}_description
    env 带 `_mujoco` 后缀（如 space_sim_mujoco）自动重定向：
        MoveIt 配置：优先 {name}_moveit_config，不存在则用 {base}_moveit_config
        描述包     ：{base}_description（几何入口 xacro/{base}.xacro；mujoco 描述包还带
                     ros2_control/mjcf，MoveIt 侧不用它）
    两处都找不到时报错并列出已构建的所有 `*_moveit_config` 包。

{base}_moveit_config 包内固定结构：
    config/{base}.srdf                                # SRDF
    config/kinematics.yaml                            # 运动学
    config/joint_limits.yaml                          # 关节限位
    config/planning_pipelines.yaml                    # 规划管线与规划器参数（OMPL / Pilz / STOMP / CHOMP）
    config/move_group_common.yaml                     # move_group 通用参数
    config/moveit.rviz                                # RViz 配置
    config/moveit_controllers_SimpleControllerManager.yaml
    config/moveit_controllers_Ros2ControlManager.yaml

use_sim_time 默认 `auto`：`_mujoco` 环境、显式列入 SIM_TIME_ENVS 的环境、以及描述包里
**没有** config/ros2_control/hardware_interface_controller_params.yaml 的纯仿真环境 -> true。

规划器（含 chomp / stomp 的参数）全部来自 config/planning_pipelines.yaml，本文件只负责原样载入。
"""

import os

import yaml

from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)
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

#: 强制使用仿真时钟的基环境。一般**不需要维护这个列表**：
#: `_mujoco` 环境、以及描述包里没有真实硬件参数文件（纯仿真，/joint_states 带仿真时间戳）的
#: 环境都会自动判为 true。这里只作为显式逃生口，留给“推断不成立但要强制仿真时钟”的环境。
SIM_TIME_ENVS = ("space_sim",)

#: 真实硬件参数文件（存在于 {env}_description/config/ros2_control/ 下表示该环境接真机）
HARDWARE_PARAMS_RELPATH = os.path.join(
    "config", "ros2_control", "hardware_interface_controller_params.yaml"
)

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


def _installed_packages():
    """ament index 里所有已构建（可 find 到）的包名。"""
    return set(get_packages_with_prefixes().keys())


def _resolve_env(env):
    """env -> (base_env, moveit_package, description_package)。

    * MoveIt 配置包：优先 `{env}_moveit_config`，不存在才回退 `{base_env}_moveit_config`
      —— 这就是“带 `_mujoco` 后缀的环境自动重定向”（space_sim_mujoco -> space_sim_moveit_config）。
    * 描述包：取**基环境**的 `{base_env}_description`（`_mujoco` 描述包含 ros2_control/mjcf，
      MoveIt 侧只用纯几何描述 `xacro/{base_env}.xacro`）。

    都找不到时抛 RuntimeError，并把已构建的 `*_moveit_config` 包列出来（省得猜环境名）。
    """
    base_env, _ = _split_env(env)
    installed = _installed_packages()

    if _moveit_package(env) in installed:
        base_env = env

    moveit_package = _moveit_package(base_env)
    if moveit_package not in installed:
        available = ", ".join(sorted(n for n in installed if n.endswith("_moveit_config")))
        # env 带 _mujoco 后缀时包名会变，报错里把回退目标也说清楚；否则不用重复
        tried = f"'{_moveit_package(env)}'"
        if base_env != env:
            tried += f" 或 '{moveit_package}'"
        raise RuntimeError(
            f"[MoveIt Launch] 找不到 MoveIt 配置包 {tried}，env='{env}'。"
            f"已构建的配置包：{available or '(无，先 colcon build)'}。"
        )

    description_package = _description_package(base_env)
    if description_package not in installed:
        raise RuntimeError(
            f"[MoveIt Launch] 找不到描述包 '{description_package}'（env='{env}'），"
            "请先 colcon build 对应环境包。"
        )

    return base_env, moveit_package, description_package


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
            # 可搬运镜片不作为机械人的 link 进规划场景（否则在初始储存位被钉死，成为
            # “幽灵镜片”）；改由任务节点用 CollisionObject 把真实网格加进世界系。
            # 仿真侧（MuJoCo）仍然用默认 true。未声明该 arg 的环境（如 lab_bench）忽略它。
            "include_movable_lenses:=false",
            " ",
        ]
    )

    return {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }


def _infer_use_sim_time(base_env, is_mujoco, description_package):
    """auto 推断：mujoco 环境 / 显式列入 SIM_TIME_ENVS / 纯仿真（无真实硬件参数文件）。

    `space_sim` 没有 config/ros2_control/hardware_interface_controller_params.yaml
    （纯仿真，/joint_states 用仿真时间戳）=> true；`lab_bench` 有该文件（真机）=> false。
    """
    if is_mujoco or base_env in SIM_TIME_ENVS:
        return True
    hardware_params = os.path.join(
        get_package_share_directory(description_package), HARDWARE_PARAMS_RELPATH
    )
    return not os.path.exists(hardware_params)


def _resolve_use_sim_time(context, base_env, is_mujoco, description_package):
    """auto/空 -> 按环境推断；显式 true/false 覆盖。"""
    raw = LaunchConfiguration("use_sim_time").perform(context).strip().lower()

    if raw in ("", "auto"):
        return _infer_use_sim_time(base_env, is_mujoco, description_package)
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
    if not env:
        raise RuntimeError("[MoveIt Launch] env 不能为空。")

    # {env}_moveit_config -> {base}_moveit_config 自动重定向（`_mujoco` 后缀）
    _, is_mujoco = _split_env(env)
    base_env, moveit_package, description_package = _resolve_env(env)

    use_sim_time = _resolve_use_sim_time(
        context, base_env, is_mujoco, description_package
    )
    use_ros2_control_manager = _resolve_use_ros2_control_manager(context)

    print(
        f"[MoveIt Launch] env={env} -> package={moveit_package}, "
        f"description={description_package}/xacro/{base_env}.xacro, "
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
                    "Environment to launch move_group for. 任意已构建的 "
                    "`{env}_moveit_config` 都可用；带 `_mujoco` 后缀的环境自动重定向到基环境"
                    "（space_sim_mujoco -> space_sim_moveit_config）。"
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
                    "auto (默认): `_mujoco` 环境、SIM_TIME_ENVS、以及描述包没有真实硬件参数"
                    "文件的纯仿真环境 -> true，其余 -> false；也可显式传 true / false。"
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
