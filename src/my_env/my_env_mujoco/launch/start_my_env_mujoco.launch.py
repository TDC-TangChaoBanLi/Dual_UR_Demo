import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


# ============================================================
# 固定工程配置：一般不通过命令行修改
# ============================================================

MUJOCO_PACKAGE = "my_env_mujoco"
CONTROL_PACKAGE = "my_env_control"
DESCRIPTION_PACKAGE = "my_env_description"

DESCRIPTION_FILE = "urdf/my_env_mujoco.urdf.xacro"
MJCF_FILE = "mjcf/my_env_mujoco.xml"
DUAL_ARM_CONFIG_FILE = "urdf/dual_arm_config.xacro"
CONTROLLERS_FILE = "config/my_env_mujoco_controller.yaml"
MUJOCO_PLUGINS_FILE = "config/mujoco_ros2_control_plugins.yaml"
PIDS_CONFIG_FILE = "config/mujoco_pids_config.yaml"
RVIZ_CONFIG_FILE = "rviz/my_env_mujoco.rviz"
SUPPORTED_GRIPPER_TYPES = {"dh_ag95", "robotiq_2f85"}


# ============================================================
# 控制器固定名称：必须与 my_env_mujoco_controller.yaml 一致
# ============================================================

# joint_state_broadcaster
# 只需要一个，它会发布总 /joint_states
BASE_ACTIVE_CONTROLLERS = [
    "joint_state_broadcaster",
]

# UR 控制器前缀
# [WARN] 必须和 controller.yaml / MoveIt 配置保持一致
UR_A_CONTROLLER_PREFIX = "arm_A_ur_"
UR_B_CONTROLLER_PREFIX = "arm_B_ur_"

# UR 仿真可用运动控制器后缀
# [WARN] 必须和 controller.yaml / MoveIt 配置保持一致
ALL_UR_CONTROLLER_SUFFIXES = [
    "joint_trajectory_controller",
    "forward_position_controller",
    "forward_velocity_controller",
    "forward_effort_controller",
]

ALL_UR_A_CONTROLLERS = [
    f"{UR_A_CONTROLLER_PREFIX}{suffix}" for suffix in ALL_UR_CONTROLLER_SUFFIXES
]
ALL_UR_B_CONTROLLERS = [
    f"{UR_B_CONTROLLER_PREFIX}{suffix}" for suffix in ALL_UR_CONTROLLER_SUFFIXES
]

# UR 仿真可用状态控制器后缀
ALL_UR_STATE_CONTROLLERS_SUFFIXES = [
    "force_torque_sensor_broadcaster",
]

ALL_UR_A_STATE_CONTROLLERS = [
    f"{UR_A_CONTROLLER_PREFIX}{suffix}" for suffix in ALL_UR_STATE_CONTROLLERS_SUFFIXES
]
ALL_UR_B_STATE_CONTROLLERS = [
    f"{UR_B_CONTROLLER_PREFIX}{suffix}" for suffix in ALL_UR_STATE_CONTROLLERS_SUFFIXES
]

# Robotiq 夹爪控制器
# [WARN] 必须和 controller.yaml / MoveIt 配置保持一致
GRIPPER_A_CONTROLLER_PREFIX = "arm_A_g2f85_"
GRIPPER_B_CONTROLLER_PREFIX = "arm_B_g2f85_"

GRIPPER_CONTROLLER_SUFFIXES = [
    "gripper_controller",
]

ALL_GRIPPER_A_CONTROLLERS = [
    f"{GRIPPER_A_CONTROLLER_PREFIX}{suffix}" for suffix in GRIPPER_CONTROLLER_SUFFIXES
]
ALL_GRIPPER_B_CONTROLLERS = [
    f"{GRIPPER_B_CONTROLLER_PREFIX}{suffix}" for suffix in GRIPPER_CONTROLLER_SUFFIXES
]

# DH-Robotics AG-95 夹爪控制器
# [WARN] 必须和 controller.yaml / MoveIt 配置保持一致
AG95_A_CONTROLLER_PREFIX = "arm_A_dhag95_"
AG95_B_CONTROLLER_PREFIX = "arm_B_dhag95_"

AG95_CONTROLLER_SUFFIXES = [
    "gripper_controller",
]

ALL_AG95_A_CONTROLLERS = [
    f"{AG95_A_CONTROLLER_PREFIX}{suffix}" for suffix in AG95_CONTROLLER_SUFFIXES
]
ALL_AG95_B_CONTROLLERS = [
    f"{AG95_B_CONTROLLER_PREFIX}{suffix}" for suffix in AG95_CONTROLLER_SUFFIXES
]


def _read_configured_gripper_type():
    """从 dual_arm_config.xacro 读取夹爪类型。"""
    config_file = (
        Path(get_package_share_directory(DESCRIPTION_PACKAGE)) / DUAL_ARM_CONFIG_FILE
    )
    root = ET.parse(config_file).getroot()

    for element in root.iter():
        if not element.tag.endswith("property"):
            continue
        if element.attrib.get("name") == "gripper_type":
            gripper_type = element.attrib.get("value")
            if gripper_type in SUPPORTED_GRIPPER_TYPES:
                return gripper_type
            supported = ", ".join(sorted(SUPPORTED_GRIPPER_TYPES))
            raise RuntimeError(
                f"Unsupported gripper_type '{gripper_type}' in {config_file}. "
                f"Supported values: {supported}."
            )

    raise RuntimeError(f"Missing gripper_type property in {config_file}.")


def launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    activate_ur_controller = LaunchConfiguration("activate_ur_controller")
    activate_ur_state_controller = LaunchConfiguration("activate_ur_state_controller")
    activate_gripper_controller = LaunchConfiguration("activate_gripper_controller")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    initial_ur_suffix = LaunchConfiguration("initial_ur_controller").perform(context)
    _gripper_type = _read_configured_gripper_type()

    # ### 激活的控制器列表 ###
    controllers_active = list(BASE_ACTIVE_CONTROLLERS)

    # 添加初始 UR 运动控制器
    ur_A_initial_controller = f"{UR_A_CONTROLLER_PREFIX}{initial_ur_suffix}"
    ur_B_initial_controller = f"{UR_B_CONTROLLER_PREFIX}{initial_ur_suffix}"
    if activate_ur_controller.perform(context).lower() == "true":
        controllers_active += [
            ur_A_initial_controller,
            ur_B_initial_controller,
        ]

    # 添加 UR 状态控制器（仿真中始终可用）
    if activate_ur_state_controller.perform(context).lower() == "true":
        controllers_active += ALL_UR_A_STATE_CONTROLLERS
        controllers_active += ALL_UR_B_STATE_CONTROLLERS

    # 添加夹爪控制器（根据 gripper_type 选择）
    if activate_gripper_controller.perform(context).lower() == "true":
        if _gripper_type == "robotiq_2f85":
            controllers_active += ALL_GRIPPER_A_CONTROLLERS
            controllers_active += ALL_GRIPPER_B_CONTROLLERS
        elif _gripper_type == "dh_ag95":
            controllers_active += ALL_AG95_A_CONTROLLERS
            controllers_active += ALL_AG95_B_CONTROLLERS

    # ### 未激活的控制器列表 ###
    controllers_inactive = []
    for controller_name in ALL_UR_A_CONTROLLERS + ALL_UR_B_CONTROLLERS:
        if controller_name not in controllers_active:
            controllers_inactive.append(controller_name)

    # ### 文件路径 ###

    description_file = PathJoinSubstitution(
        [FindPackageShare(MUJOCO_PACKAGE), DESCRIPTION_FILE]
    )

    mjcf_file = PathJoinSubstitution(
        [FindPackageShare(MUJOCO_PACKAGE), MJCF_FILE]
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(MUJOCO_PACKAGE), CONTROLLERS_FILE]
    )

    mujoco_plugins_file = PathJoinSubstitution(
        [FindPackageShare(MUJOCO_PACKAGE), MUJOCO_PLUGINS_FILE]
    )

    pids_config_file = PathJoinSubstitution(
        [FindPackageShare(MUJOCO_PACKAGE), PIDS_CONFIG_FILE]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(CONTROL_PACKAGE), RVIZ_CONFIG_FILE]
    )

    # ### robot_description ###

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "mjcf_file_path:=", mjcf_file, " ",
            "pids_config_file_path:=", pids_config_file, " ",
            "camera_use_nominal_extrinsics:=false",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 1. robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
            {"publish_frequency": 100.0},
        ],
    )

    # 2. ros2_control_node（使用 mujoco_ros2_control 包）
    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controllers_file, allow_substs=True),
            ParameterFile(mujoco_plugins_file),
        ],
        remappings=(
            [("~/robot_description", "/robot_description")]
            if os.environ.get("ROS_DISTRO") == "humble"
            else []
        ),
        on_exit=Shutdown(),
    )

    nodes = [
        robot_state_publisher_node,
        control_node,
    ]

    # 3. 控制器 spawner 辅助函数
    def controller_spawner(controller_names, active=True):
        inactive_flag = ["--inactive"] if not active else []

        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers_file, allow_substs=True),
            ],
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flag
            + controller_names,
        )

    # 4. 激活控制器 spawner
    active_controller_spawner = controller_spawner(controllers_active, active=True)
    nodes.append(active_controller_spawner)

    # 5. 未激活控制器 spawner
    if controllers_inactive:
        inactive_controller_spawner = controller_spawner(
            controllers_inactive, active=False
        )
        nodes.append(inactive_controller_spawner)

    # 6. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )
    nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_ur_controller",
            default_value="forward_position_controller",
            choices=ALL_UR_CONTROLLER_SUFFIXES,
            description=(
                "Initial UR controller suffix. "
                "The launch file will expand it to arm_A_ur_* and arm_B_ur_*."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_ur_controller",
            default_value="true",
            description="Activate the selected initial UR motion controllers.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_ur_state_controller",
            default_value="true",
            description="Activate the UR state broadcasters (FTS, TCP pose).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_gripper_controller",
            default_value="true",
            description="Activate the gripper controllers.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="15",
            description="Timeout used by controller_manager spawner.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz.",
        )
    )


    # Note: xacro_file_path is declared as an argument for backward
    # compatibility and manual override. The default comes from
    # DESCRIPTION_FILE constant above.

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
