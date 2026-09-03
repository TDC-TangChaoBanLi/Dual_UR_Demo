#!/usr/bin/env python3
"""
启动双臂 UR OCS2 控制器 + Interactive Marker + RViz

用法:
  ros2 launch launch_controller start_ocs2_arms_controller.launch.py env:=lab_bench use_fake_hardware:=true
  ros2 launch launch_controller start_ocs2_arms_controller.launch.py env:=lab_bench_mujoco mujoco_headless:=true launch_rviz:=false
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


# ============================================================
# 环境包命名规则：{env}_description
# 每个环境包内固定结构：
#   xacro/ros2_control/{env}.xacro          # ros2_control 入口
#   config/ros2_control/ocs2_arms_controllers.yaml
#   rviz/{env}_ocs2_arms.rviz
#   mujoco 环境额外：
#     config/mujoco_ros2_control_plugins.yaml
#     config/mujoco_pids_config.yaml
#     mjcf/{env}.xml
# ============================================================

# 与 start_traditional_controllers.launch.py 保持一致（同一 planning URDF 路径）
PLANNING_URDF_PATH = "/tmp/dual_ur_ocs2_planning.urdf"

# 支持的夹爪类型（与 start_traditional_controllers.launch.py 保持一致）
SUPPORTED_GRIPPER_TYPES = {"dh_ag95", "robotiq_2f85", "none"}

# 夹爪控制器（已按 left/right 命名，与 GripperControlPanel / VR 匹配）
# 仅当环境配置了夹爪（gripper_type != none）时启用
HAND_CONTROLLERS = [
    "left_adaptive_gripper_controller",
    "right_adaptive_gripper_controller",
]

# 目标管理器基座帧（所有环境统一使用 base_link）
CONTROL_BASE_FRAME = "base_link"


def _env_package(env):
    """环境包名：{env}_description。"""
    return f"{env}_description"


def _is_mujoco_env(env):
    """通过包内是否存在 mujoco 插件配置判断是否为 mujoco 环境。"""
    share = get_package_share_directory(_env_package(env))
    return os.path.exists(
        os.path.join(share, "config", "mujoco_ros2_control_plugins.yaml")
    )


def _read_configured_gripper_type(env, is_mujoco):
    """从 dual_arm_config.xacro 读取夹爪类型。

    mujoco 环境复用对应非 mujoco 环境的几何描述
    （space_sim_mujoco -> space_sim_description）。
    """
    if is_mujoco:
        config_package = env.replace("_mujoco", "") + "_description"
    else:
        config_package = _env_package(env)

    config_path = (
        Path(get_package_share_directory(config_package)) / "xacro" / "dual_arm_config.xacro"
    )
    root = ET.parse(config_path).getroot()

    for element in root.iter():
        if not element.tag.endswith("property"):
            continue
        if element.attrib.get("name") == "gripper_type":
            gripper_type = element.attrib.get("value")
            if gripper_type in SUPPORTED_GRIPPER_TYPES:
                return gripper_type
            supported = ", ".join(sorted(SUPPORTED_GRIPPER_TYPES))
            raise RuntimeError(
                f"Unsupported gripper_type '{gripper_type}' in {config_path}. "
                f"Supported values: {supported}."
            )

    raise RuntimeError(f"Missing gripper_type property in {config_path}.")


def _get_hand_controllers(env, is_mujoco):
    """根据环境夹爪类型返回夹爪控制器列表（无夹爪时为空）。"""
    gripper_type = _read_configured_gripper_type(env, is_mujoco)
    if gripper_type == "none":
        return []
    return list(HAND_CONTROLLERS)


def _get_control_base_frame(env):
    """返回环境对应的目标管理器基座帧（所有环境统一 base_link）。"""
    return CONTROL_BASE_FRAME


def _load_hardware_params(env):
    """读取统一硬件接口参数配置文件（传统模式）。

    文件为扁平结构 {参数名}:{值}，直接作为 xacro 参数传入。
    文件不存在时返回空 dict（如 space_sim 无真实硬件）。
    """
    config_file = os.path.join(
        get_package_share_directory(_env_package(env)),
        "config", "ros2_control", "hardware_interface_controller_params.yaml",
    )
    if not os.path.exists(config_file):
        return {}
    with open(config_file, "r") as f:
        return yaml.safe_load(f) or {}


def _build_xacro_cmd(env, is_mujoco, context):
    """构建 xacro 命令行参数列表。"""
    share = get_package_share_directory(_env_package(env))
    xacro_input = os.path.join(share, "xacro", "ros2_control", f"{env}.xacro")

    if is_mujoco:
        mjcf_file = os.path.join(share, "mjcf", f"{env}.xml")
        pids_file = os.path.join(share, "config", "mujoco_pids_config.yaml")
        headless = LaunchConfiguration("mujoco_headless").perform(context)
        sim_speed_factor = LaunchConfiguration("mujoco_sim_speed_factor").perform(context)
        return [
            "xacro", xacro_input,
            "mjcf_file_path:=" + mjcf_file,
            "pids_config_file_path:=" + pids_file,
            "camera_use_nominal_extrinsics:=false",
            "mujoco_sim_speed_factor:=" + sim_speed_factor,
            "mujoco_headless:=" + headless,
        ]

    fake_hw = LaunchConfiguration("use_fake_hardware").perform(context)
    headless = LaunchConfiguration("ur_headless_mode").perform(context)
    HW = _load_hardware_params(env)
    args = [
        "xacro", xacro_input,
        "use_fake_hardware:=" + fake_hw,
        "ur_headless_mode:=" + headless,
    ]
    # 将硬件参数文件中的 {参数名}:{值} 逐个作为 xacro 参数传入
    # 不同环境可定义不同数量/名称的硬件参数
    for k, v in HW.items():
        args.append(f"{k}:={v}")
    return args


def _generate_planning_urdf(urdf_xml: str, output_path: str) -> str:
    """从完整 URDF XML 中移除 <ros2_control> 标签（供 OCS2 Pinocchio 加载）。"""
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError:
        Path(output_path).write_text(urdf_xml)
        return output_path

    parent_map = {c: p for p in root.iter() for c in list(p)}
    for child in list(root.iter()):
        if child.tag in ("ros2_control",) or child.tag.endswith("}ros2_control"):
            parent = parent_map.get(child)
            if parent is not None:
                parent.remove(child)

    Path(output_path).write_text(ET.tostring(root, encoding="unicode"))
    return output_path


# ============================================================
def launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration("env").perform(context)
    is_mujoco = _is_mujoco_env(env)
    launch_rviz = LaunchConfiguration("launch_rviz")

    # ========== 运行 xacro 一次，同时用于 robot_description 和 planning URDF ==========
    print(f"[OCS2 Launch] Running xacro (env={env})...")
    result = subprocess.run(
        _build_xacro_cmd(env, is_mujoco, context),
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"[OCS2 Launch] xacro error:\n{result.stderr}")
        sys.exit(1)

    robot_description = {"robot_description": result.stdout}

    _generate_planning_urdf(result.stdout, PLANNING_URDF_PATH)
    print(f"[OCS2 Launch] Planning URDF -> {PLANNING_URDF_PATH}")

    # ========== 文件路径（{env}_description 包内固定结构） ==========
    controllers_path = os.path.join(
        get_package_share_directory(_env_package(env)),
        "config", "ros2_control", "ocs2_arms_controllers.yaml",
    )
    rviz_path = os.path.join(
        get_package_share_directory(_env_package(env)),
        "rviz", f"{env}_ocs2_arms.rviz",
    )

    use_sim_time = True if is_mujoco else (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )

    # 根据环境夹爪类型决定夹爪控制器列表（无夹爪时为空）
    hand_controllers = _get_hand_controllers(env, is_mujoco)
    # 目标管理器基座帧（lab_bench -> arm_shelf, space_sim -> base_cube）
    control_base_frame = _get_control_base_frame(env)

    # 1. robot_state_publisher（仿真必须同步时钟）
    robot_state_publisher_params = [robot_description, {"use_sim_time": use_sim_time}]
    if is_mujoco:
        robot_state_publisher_params.append({"publish_frequency": 100.0})

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=robot_state_publisher_params),
    ]

    # 2. ros2_control_node（mujoco 使用 mujoco_ros2_control 包）
    if is_mujoco:
        mujoco_share = get_package_share_directory(_env_package(env))
        mujoco_plugins_path = os.path.join(mujoco_share, "config", "mujoco_ros2_control_plugins.yaml")
        nodes.append(
            Node(package="mujoco_ros2_control", executable="ros2_control_node",
                 output="screen",
                 parameters=[robot_description,
                             {"use_sim_time": True},
                             ParameterFile(controllers_path, allow_substs=True),
                             ParameterFile(mujoco_plugins_path)],
                 remappings=(
                     [("~/robot_description", "/robot_description")]
                     if os.environ.get("ROS_DISTRO") == "humble"
                     else []
                 ),
                 on_exit=Shutdown()),
        )
    else:
        nodes.append(
            Node(package="controller_manager", executable="ros2_control_node",
                 output="screen",
                 parameters=[robot_description,
                             {"use_sim_time": use_sim_time},
                             ParameterFile(controllers_path, allow_substs=True)]),
        )

    # 3. 控制器 spawner
    nodes.append(
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "60",
                        "joint_state_broadcaster", "ocs2_arm_controller"]
                    + hand_controllers),
    )

    # 4. Interactive Marker 目标管理器（帧与 OCS2 launch 一致）
    # 注意：hand_controllers 为空（如 space_sim 无夹爪）时不能传空列表，
    # launch_ros 参数评估会因空 tuple 报错，因此仅在非空时传入该参数。
    target_manager_params = {
        "dual_arm_mode": True,
        "control_base_frame": control_base_frame,
        "marker_fixed_frame": control_base_frame,
        "linear_scale": 0.005, "angular_scale": 0.05,
        "enable_vr": False, "use_sim_time": use_sim_time,
    }
    if hand_controllers:
        target_manager_params["hand_controllers"] = hand_controllers
    nodes.append(
        Node(package="arms_target_manager", executable="arms_target_manager_node",
             output="screen", parameters=[target_manager_params]),
    )

    if launch_rviz.perform(context).lower() != "false":
        rviz_params = [{"use_sim_time": use_sim_time}]
        if hand_controllers:
            # GripperControlPanel 通过 rviz 节点的 hand_controllers
            # 参数自动发现夹爪控制器（无此参数时面板为空）。
            rviz_params.append({"hand_controllers": hand_controllers})
        nodes.append(
            Node(package="rviz2", executable="rviz2", output="log",
                 arguments=["-d", rviz_path],
                 parameters=rviz_params),
        )

    return nodes


# ============================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "env",
            default_value="lab_bench",
            description="Environment to launch OCS2 controllers for (e.g. lab_bench, lab_bench_mujoco).",
        ),
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="Use mock hardware (traditional env only)."),
        DeclareLaunchArgument("ur_headless_mode", default_value="false",
                              description="Enable UR headless mode (traditional env only)."),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz."),
        # 默认 false：fake hardware 场景无 /clock 发布者，使用 wall clock。
        # 仿真（MuJoCo 等，有 /clock）或需要同步仿真时钟时传 use_sim_time:=true。
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation clock (traditional env only)."),
        DeclareLaunchArgument("mujoco_headless", default_value="false",
                              description="Run MuJoCo without the Simulate viewer window (mujoco env only)."),
        DeclareLaunchArgument("mujoco_sim_speed_factor", default_value="1.0",
                              description="Simulation speed factor (mujoco env only)."),
        OpaqueFunction(function=launch_setup),
    ])