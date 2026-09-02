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

# 夹爪控制器（已按 left/right 命名，与 GripperControlPanel / VR 匹配）
HAND_CONTROLLERS = [
    "left_adaptive_gripper_controller",
    "right_adaptive_gripper_controller",
]


def _env_package(env):
    """环境包名：{env}_description。"""
    return f"{env}_description"


def _is_mujoco_env(env):
    """通过包内是否存在 mujoco 插件配置判断是否为 mujoco 环境。"""
    share = get_package_share_directory(_env_package(env))
    return os.path.exists(
        os.path.join(share, "config", "mujoco_ros2_control_plugins.yaml")
    )


def _load_hardware_params(env):
    """读取统一硬件接口参数配置文件（传统模式）。"""
    config_file = os.path.join(
        get_package_share_directory(_env_package(env)),
        "config", "ros2_control", "hardware_interface_controller_params.yaml",
    )
    with open(config_file, "r") as f:
        return yaml.safe_load(f)


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
    ur = HW["ur"]
    g2 = HW["g2f85"]
    ag = HW["ag95"]
    return [
        "xacro", xacro_input,
        "use_fake_hardware:=" + fake_hw,
        "ur_headless_mode:=" + headless,
        "ur_reverse_ip:=" + ur["reverse_ip"],
        "ur_A_robot_ip:=" + ur["arm_A"]["robot_ip"],
        "ur_B_robot_ip:=" + ur["arm_B"]["robot_ip"],
        "ur_A_reverse_port:=" + ur["arm_A"]["reverse_port"],
        "ur_A_script_sender_port:=" + ur["arm_A"]["script_sender_port"],
        "ur_A_script_command_port:=" + ur["arm_A"]["script_command_port"],
        "ur_A_trajectory_port:=" + ur["arm_A"]["trajectory_port"],
        "ur_A_rw_rate:=" + ur["arm_A"]["rw_rate"],
        "ur_B_reverse_port:=" + ur["arm_B"]["reverse_port"],
        "ur_B_script_sender_port:=" + ur["arm_B"]["script_sender_port"],
        "ur_B_script_command_port:=" + ur["arm_B"]["script_command_port"],
        "ur_B_trajectory_port:=" + ur["arm_B"]["trajectory_port"],
        "ur_B_rw_rate:=" + ur["arm_B"]["rw_rate"],
        "g2f85_A_com_port:=" + g2["A_com_port"],
        "g2f85_B_com_port:=" + g2["B_com_port"],
        "ag95_gripper_transport_type:=" + ag["transport_type"],
        "ag95_gripper_serial_baudrate:=" + ag["serial_baudrate"],
        "ag95_gripper_can_bitrate:=" + ag["can_bitrate"],
        "ag95_gripper_pcan_bitrate:=" + ag["pcan_bitrate"],
        "ag95_gripper_gripper_model:=" + ag["gripper_model"],
        "ag95_gripper_default_force_percent:=" + ag["default_force_percent"],
        "ag95_gripper_auto_initialize:=" + ag["auto_initialize"],
        "ag95_gripper_rw_rate:=" + ag["rw_rate"],
        "ag95_gripper_command_interval_ms:=" + ag["command_interval_ms"],
        "ag95_A_serial_port:=" + ag["arm_A"]["serial_port"],
        "ag95_A_can_interface:=" + ag["arm_A"]["can_interface"],
        "ag95_A_pcan_channel:=" + ag["arm_A"]["pcan_channel"],
        "ag95_A_gripper_id:=" + ag["arm_A"]["gripper_id"],
        "ag95_B_serial_port:=" + ag["arm_B"]["serial_port"],
        "ag95_B_can_interface:=" + ag["arm_B"]["can_interface"],
        "ag95_B_pcan_channel:=" + ag["arm_B"]["pcan_channel"],
        "ag95_B_gripper_id:=" + ag["arm_B"]["gripper_id"],
    ]


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
                    + HAND_CONTROLLERS),
    )

    # 4. Interactive Marker 目标管理器（帧与 OCS2 launch 一致：arm_shelf）
    nodes.append(
        Node(package="arms_target_manager", executable="arms_target_manager_node",
             output="screen", parameters=[{
                 "dual_arm_mode": True,
                 "control_base_frame": "arm_shelf",
                 "marker_fixed_frame": "arm_shelf",
                 "hand_controllers": HAND_CONTROLLERS,
                 "linear_scale": 0.005, "angular_scale": 0.05,
                 "enable_vr": False, "use_sim_time": use_sim_time,
             }]),
    )

    if launch_rviz.perform(context).lower() != "false":
        nodes.append(
            Node(package="rviz2", executable="rviz2", output="log",
                 arguments=["-d", rviz_path],
                 parameters=[{"use_sim_time": use_sim_time},
                             # GripperControlPanel 通过 rviz 节点的 hand_controllers
                             # 参数自动发现夹爪控制器（无此参数时面板为空）。
                             {"hand_controllers": HAND_CONTROLLERS}]),
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