#!/usr/bin/env python3
"""
启动双臂 UR OCS2 控制器 + Interactive Marker + RViz

用法:
  ros2 launch my_env_control start_my_env_ocs2.launch.py use_fake_hardware:=true
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


# ============================================================
CONTROL_PACKAGE = "my_env_control"
DESCRIPTION_FILE = "urdf/my_env_control.urdf.xacro"
OCS2_CONTROLLERS_FILE = "config/dual_arm_ocs2_controllers.yaml"
OCS2_RVIZ_FILE = "rviz/my_env_ocs2.rviz"

# ---- 硬件网络常量（与 start_my_env_control.launch.py 保持一致） ----
UR_REVERSE_IP = "192.168.8.100"
UR_A_ROBOT_IP = "192.168.8.17"
UR_B_ROBOT_IP = "192.168.8.11"
UR_A_REVERSE_PORT = "50001"; UR_A_SCRIPT_SENDER_PORT = "50002"
UR_A_SCRIPT_COMMAND_PORT = "50003"; UR_A_TRAJECTORY_PORT = "50004"
UR_B_REVERSE_PORT = "50011"; UR_B_SCRIPT_SENDER_PORT = "50012"
UR_B_SCRIPT_COMMAND_PORT = "50013"; UR_B_TRAJECTORY_PORT = "50014"
UR_A_RW_RATE = "125"; UR_B_RW_RATE = "500"
G2F85_A_COM_PORT = "/dev/ttyUSB0"; G2F85_B_COM_PORT = "/dev/ttyUSB1"

# ---- DH-Robotics AG-95 ----
AG95_GRIPPER_TRANSPORT_TYPE = "socketcan"
AG95_GRIPPER_SERIAL_BAUDRATE = "115200"
AG95_GRIPPER_CAN_BITRATE = "500000"
AG95_GRIPPER_PCAN_BITRATE = "500000"
AG95_GRIPPER_GRIPPER_MODEL = "ag-160-95"
AG95_GRIPPER_DEFAULT_FORCE_PERCENT = "100"
AG95_GRIPPER_AUTO_INITIALIZE = "true"
AG95_GRIPPER_RW_RATE = "25"
AG95_GRIPPER_COMMAND_INTERVAL_MS = "10"
AG95_A_SERIAL_PORT = "/dev/ttyAG95_A"
AG95_A_CAN_INTERFACE = "can0"; AG95_A_PCAN_CHANNEL = "PCAN_USBBUS1"
AG95_A_GRIPPER_ID = "1"
AG95_B_SERIAL_PORT = "/dev/ttyAG95_B"
AG95_B_CAN_INTERFACE = "can0"; AG95_B_PCAN_CHANNEL = "PCAN_USBBUS1"
AG95_B_GRIPPER_ID = "2"


def _build_xacro_cmd(xacro_input, fake_hw, fake_sensor, headless, ur_a_init, ur_b_init):
    """构建 xacro 命令行参数列表（单一定义，避免重复）。"""
    return [
        "xacro", xacro_input,
        "use_fake_hardware:=" + fake_hw,
        "use_fake_sensor_commands:=" + fake_sensor,
        "ur_headless_mode:=" + headless,
        "ur_reverse_ip:=" + UR_REVERSE_IP,
        "ur_A_robot_ip:=" + UR_A_ROBOT_IP,
        "ur_B_robot_ip:=" + UR_B_ROBOT_IP,
        "ur_A_reverse_port:=" + UR_A_REVERSE_PORT,
        "ur_A_script_sender_port:=" + UR_A_SCRIPT_SENDER_PORT,
        "ur_A_script_command_port:=" + UR_A_SCRIPT_COMMAND_PORT,
        "ur_A_trajectory_port:=" + UR_A_TRAJECTORY_PORT,
        "ur_A_rw_rate:=" + UR_A_RW_RATE,
        "ur_A_initial_positions_file:=" + ur_a_init,
        "ur_B_reverse_port:=" + UR_B_REVERSE_PORT,
        "ur_B_script_sender_port:=" + UR_B_SCRIPT_SENDER_PORT,
        "ur_B_script_command_port:=" + UR_B_SCRIPT_COMMAND_PORT,
        "ur_B_trajectory_port:=" + UR_B_TRAJECTORY_PORT,
        "ur_B_rw_rate:=" + UR_B_RW_RATE,
        "ur_B_initial_positions_file:=" + ur_b_init,
        "g2f85_A_com_port:=" + G2F85_A_COM_PORT,
        "g2f85_B_com_port:=" + G2F85_B_COM_PORT,
        "ag95_gripper_transport_type:=" + AG95_GRIPPER_TRANSPORT_TYPE,
        "ag95_gripper_serial_baudrate:=" + AG95_GRIPPER_SERIAL_BAUDRATE,
        "ag95_gripper_can_bitrate:=" + AG95_GRIPPER_CAN_BITRATE,
        "ag95_gripper_pcan_bitrate:=" + AG95_GRIPPER_PCAN_BITRATE,
        "ag95_gripper_gripper_model:=" + AG95_GRIPPER_GRIPPER_MODEL,
        "ag95_gripper_default_force_percent:=" + AG95_GRIPPER_DEFAULT_FORCE_PERCENT,
        "ag95_gripper_auto_initialize:=" + AG95_GRIPPER_AUTO_INITIALIZE,
        "ag95_gripper_rw_rate:=" + AG95_GRIPPER_RW_RATE,
        "ag95_gripper_command_interval_ms:=" + AG95_GRIPPER_COMMAND_INTERVAL_MS,
        "ag95_A_serial_port:=" + AG95_A_SERIAL_PORT,
        "ag95_A_can_interface:=" + AG95_A_CAN_INTERFACE,
        "ag95_A_pcan_channel:=" + AG95_A_PCAN_CHANNEL,
        "ag95_A_gripper_id:=" + AG95_A_GRIPPER_ID,
        "ag95_B_serial_port:=" + AG95_B_SERIAL_PORT,
        "ag95_B_can_interface:=" + AG95_B_CAN_INTERFACE,
        "ag95_B_pcan_channel:=" + AG95_B_PCAN_CHANNEL,
        "ag95_B_gripper_id:=" + AG95_B_GRIPPER_ID,
    ]


def _generate_planning_urdf(urdf_xml: str, output_path: str) -> str:
    """从完整 URDF XML 中移除 <ros2_control> 标签。"""
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
    fake_hw = LaunchConfiguration("use_fake_hardware").perform(context)
    fake_sensor = LaunchConfiguration("use_fake_sensor_commands").perform(context)
    headless = LaunchConfiguration("ur_headless_mode").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")

    control_share = get_package_share_directory(CONTROL_PACKAGE)
    xacro_input = os.path.join(control_share, DESCRIPTION_FILE)
    ur_a_init = os.path.join(control_share, "config", "ur_A_initial_positions.yaml")
    ur_b_init = os.path.join(control_share, "config", "ur_B_initial_positions.yaml")

    # ========== 运行 xacro 一次，同时用于 robot_description 和 planning URDF ==========
    print("[OCS2 Launch] Running xacro...")
    result = subprocess.run(
        _build_xacro_cmd(xacro_input, fake_hw, fake_sensor, headless, ur_a_init, ur_b_init),
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"[OCS2 Launch] xacro error:\n{result.stderr}")
        sys.exit(1)

    robot_description = {"robot_description": result.stdout}

    planning_urdf_path = "/tmp/dual_ur_ocs2_planning.urdf"
    _generate_planning_urdf(result.stdout, planning_urdf_path)
    print(f"[OCS2 Launch] Planning URDF -> {planning_urdf_path}")

    # ========== 节点 ==========
    controllers_path = os.path.join(control_share, OCS2_CONTROLLERS_FILE)
    rviz_path = os.path.join(control_share, OCS2_RVIZ_FILE)

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[robot_description]),

        Node(package="controller_manager", executable="ros2_control_node",
             output="screen",
             parameters=[robot_description,
                        ParameterFile(controllers_path, allow_substs=True)]),

        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "60",
                        "joint_state_broadcaster", "ocs2_arm_controller",
                        "arm_A_adaptive_gripper_controller",
                        "arm_B_adaptive_gripper_controller"]),

        Node(package="arms_target_manager", executable="arms_target_manager_node",
             output="screen", parameters=[{
                 "dual_arm_mode": True,
                 "control_base_frame": "arm_shelf",
                 "marker_fixed_frame": "arm_shelf",
                 "hand_controllers": ["arm_A_adaptive_gripper_controller",
                                      "arm_B_adaptive_gripper_controller"],
                 "linear_scale": 0.005, "angular_scale": 0.05,
                 "enable_vr": False, "use_sim_time": True,
             }], condition=UnlessCondition(LaunchConfiguration("no_arms_target_manager"))),
    ]

    if launch_rviz.perform(context).lower() != "false":
        nodes.append(
            Node(package="rviz2", executable="rviz2", output="log",
                 arguments=["-d", rviz_path],
                 parameters=[{"use_sim_time": True}],
                 condition=UnlessCondition(LaunchConfiguration("no_rviz"))),
        )

    return nodes


# ============================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("use_fake_sensor_commands", default_value="true"),
        DeclareLaunchArgument("ur_headless_mode", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("no_rviz", default_value="false"),
        DeclareLaunchArgument("no_arms_target_manager", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
