#!/usr/bin/env python3
"""
启动双臂 UR OCS2 控制器 + Interactive Marker + RViz（MuJoCo 仿真版）

用法:
  ros2 launch my_env_mujoco start_my_env_mujoco_ocs2.launch.py
  ros2 launch my_env_mujoco start_my_env_mujoco_ocs2.launch.py mujoco_headless:=true launch_rviz:=false
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


# ============================================================
MUJOCO_PACKAGE = "my_env_mujoco"
CONTROL_PACKAGE = "my_env_control"

DESCRIPTION_FILE = "urdf/my_env_mujoco.urdf.xacro"
MJCF_FILE = "mjcf/my_env_mujoco.xml"
MUJOCO_PLUGINS_FILE = "config/mujoco_ros2_control_plugins.yaml"
PIDS_CONFIG_FILE = "config/mujoco_pids_config.yaml"

# 复用 my_env_control 中的 OCS2 控制器配置与 RViz 配置
OCS2_CONTROLLERS_FILE = "config/dual_arm_ocs2_controllers.yaml"
OCS2_RVIZ_FILE = "rviz/my_env_ocs2.rviz"

# 与 start_my_env_ocs2.launch.py 保持一致（同一 planning URDF 路径）
PLANNING_URDF_PATH = "/tmp/dual_ur_ocs2_planning.urdf"

# 夹爪控制器（已按 left/right 命名，与 GripperControlPanel / VR 匹配）
HAND_CONTROLLERS = [
    "left_adaptive_gripper_controller",
    "right_adaptive_gripper_controller",
]


def _build_xacro_cmd(xacro_input, headless, sim_speed_factor):
    """构建 MuJoCo xacro 命令行参数列表。"""
    mujoco_share = get_package_share_directory(MUJOCO_PACKAGE)
    mjcf_file = os.path.join(mujoco_share, MJCF_FILE)
    pids_file = os.path.join(mujoco_share, PIDS_CONFIG_FILE)
    return [
        "xacro", xacro_input,
        "mjcf_file_path:=" + mjcf_file,
        "pids_config_file_path:=" + pids_file,
        "camera_use_nominal_extrinsics:=false",
        "mujoco_sim_speed_factor:=" + sim_speed_factor,
        "mujoco_headless:=" + headless,
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
    headless = LaunchConfiguration("mujoco_headless").perform(context)
    sim_speed_factor = LaunchConfiguration("mujoco_sim_speed_factor").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")

    mujoco_share = get_package_share_directory(MUJOCO_PACKAGE)
    control_share = get_package_share_directory(CONTROL_PACKAGE)
    xacro_input = os.path.join(mujoco_share, DESCRIPTION_FILE)

    # ========== 运行 xacro 一次，同时用于 robot_description 和 planning URDF ==========
    print("[MuJoCo OCS2 Launch] Running xacro...")
    result = subprocess.run(
        _build_xacro_cmd(xacro_input, headless, sim_speed_factor),
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"[MuJoCo OCS2 Launch] xacro error:\n{result.stderr}")
        sys.exit(1)

    robot_description = {"robot_description": result.stdout}

    _generate_planning_urdf(result.stdout, PLANNING_URDF_PATH)
    print(f"[MuJoCo OCS2 Launch] Planning URDF -> {PLANNING_URDF_PATH}")

    # ========== 文件路径 ==========
    controllers_path = os.path.join(control_share, OCS2_CONTROLLERS_FILE)
    mujoco_plugins_path = os.path.join(mujoco_share, MUJOCO_PLUGINS_FILE)
    rviz_path = os.path.join(control_share, OCS2_RVIZ_FILE)

    nodes = [
        # 1. robot_state_publisher（仿真必须同步时钟）
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[robot_description,
                         {"use_sim_time": True},
                         {"publish_frequency": 100.0}]),

        # 2. MuJoCo ros2_control 节点（内嵌模拟器）
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

        # 3. 控制器 spawner
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "60",
                        "joint_state_broadcaster", "ocs2_arm_controller"]
                    + HAND_CONTROLLERS),

        # 4. Interactive Marker 目标管理器（帧与 OCS2 launch 一致：arm_shelf）
        Node(package="arms_target_manager", executable="arms_target_manager_node",
             output="screen", parameters=[{
                 "dual_arm_mode": True,
                 "control_base_frame": "arm_shelf",
                 "marker_fixed_frame": "arm_shelf",
                 "hand_controllers": HAND_CONTROLLERS,
                 "linear_scale": 0.005, "angular_scale": 0.05,
                 "enable_vr": False, "use_sim_time": True,
             }]),
    ]

    if launch_rviz.perform(context).lower() != "false":
        nodes.append(
            Node(package="rviz2", executable="rviz2", output="log",
                 arguments=["-d", rviz_path],
                 parameters=[{"use_sim_time": True},
                             # GripperControlPanel 通过 rviz 节点的 hand_controllers
                             # 参数自动发现夹爪控制器（无此参数时面板为空）。
                             {"hand_controllers": HAND_CONTROLLERS}]),
        )

    return nodes


# ============================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mujoco_headless", default_value="false",
                              description="Run MuJoCo without the Simulate viewer window."),
        DeclareLaunchArgument("mujoco_sim_speed_factor", default_value="1.0",
                              description="Simulation speed factor (1.0 = realtime)."),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz."),
        OpaqueFunction(function=launch_setup),
    ])
