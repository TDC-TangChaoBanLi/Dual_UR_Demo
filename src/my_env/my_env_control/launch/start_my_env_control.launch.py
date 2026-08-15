from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
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

CONTROL_PACKAGE = "my_env_control"
DESCRIPTION_PACKAGE = "my_env_description"

DESCRIPTION_FILE = "urdf/my_env_control.urdf.xacro"
DUAL_ARM_CONFIG_FILE = "urdf/dual_arm_config.xacro"
CONTROLLERS_FILE = "config/dual_arm_controllers.yaml"
RVIZ_CONFIG_FILE = "rviz/my_env_control.rviz"
SUPPORTED_GRIPPER_TYPES = {"dh_ag95", "robotiq_2f85"}


def _load_hardware_params():
    """读取统一硬件接口参数配置文件。

    硬件参数（UR 网络 / Robotiq 串口 / DH-AG95 夹爪）集中定义在
    config/hardware_interface_controller_params.yaml，供两个 launch 文件共用。
    """
    config_file = (
        Path(get_package_share_directory(CONTROL_PACKAGE))
        / "config" / "hardware_interface_controller_params.yaml"
    )
    with open(config_file, "r") as f:
        return yaml.safe_load(f)


# 是否启动真实 UR 辅助节点
LAUNCH_URSCRIPT_INTERFACE = False
LAUNCH_ROBOT_STATE_HELPER = False

# 说明：
# controller_stopper_node 在“单全局 controller_manager + 双 UR”的场景下需要非常小心，
# 否则某一台 UR program 停止时可能影响另一台 UR 的 active controller。
# 第一版建议先不启动，等双臂主链路稳定后再单独处理。
LAUNCH_CONTROLLER_STOPPER = False


# ============================================================
# 控制器固定名称：必须与 dual_ur_g2f85_controllers.yaml 一致
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

# UR 所有可选控制器后缀
# [WARN] 必须和 controller.yaml / MoveIt 配置保持一致
ALL_UR_CONTROLLER_SUFFIXES = [
    "scaled_joint_trajectory_controller",
    "joint_trajectory_controller",
    "passthrough_trajectory_controller",
    "forward_position_controller",
    "forward_velocity_controller",
    "forward_effort_controller",
    "force_mode_controller",
    "freedrive_mode_controller",
]
# 非初始的 UR 命令控制器会加载为 inactive
ALL_UR_A_CONTROLLERS = [
    f"{UR_A_CONTROLLER_PREFIX}{suffix}" for suffix in ALL_UR_CONTROLLER_SUFFIXES
]
ALL_UR_B_CONTROLLERS = [
    f"{UR_B_CONTROLLER_PREFIX}{suffix}" for suffix in ALL_UR_CONTROLLER_SUFFIXES
]

# 如果你的 controller.yaml 中定义了 UR 状态类控制器，可以在这里打开。
# 这些控制器对 MoveIt 不是必须，但对真实 UR 状态监控有用。
ALL_UR_STATE_CONTROLLERS_SUFFIXES = [
    "io_and_status_controller",
    "speed_scaling_state_broadcaster",
    "force_torque_sensor_broadcaster",
    "tcp_pose_broadcaster",
    "ur_configuration_controller",
]

ALL_UR_A_STATE_CONTROLLERS = [
    f"{UR_A_CONTROLLER_PREFIX}{suffix}" for suffix in ALL_UR_STATE_CONTROLLERS_SUFFIXES
]
ALL_UR_B_STATE_CONTROLLERS = [
    f"{UR_B_CONTROLLER_PREFIX}{suffix}" for suffix in ALL_UR_STATE_CONTROLLERS_SUFFIXES
]


# Robotiq 控制器前缀
# [WARN] 必须和 controller.yaml / MoveIt 配置保持一致
GRIPPER_A_CONTROLLER_PREFIX = "arm_A_g2f85_"
GRIPPER_B_CONTROLLER_PREFIX = "arm_B_g2f85_"

# DH-Robotics AG-95 控制器前缀
# [WARN] 必须和 controller.yaml / MoveIt 配置保持一致
AG95_A_CONTROLLER_PREFIX = "arm_A_dhag95_"
AG95_B_CONTROLLER_PREFIX = "arm_B_dhag95_"

# 双 Robotiq 控制器名称后缀
# [WARN] 必须与你的 controller.yaml 一致。

# ros2 内部夹爪控制器后缀
INNER_GRIPPER_CONTROLLERS_SUFFIXES = [
    "gripper_controller",
]
# 外部包夹爪控制器后缀
EXTERNAL_GRIPPER_CONTROLLERS_SUFFIXES = [
    "activation_controller",
]

ALL_GRIPPER_INNER_CONTROLLERS = [
    f"{GRIPPER_A_CONTROLLER_PREFIX}{suffix}" for suffix in INNER_GRIPPER_CONTROLLERS_SUFFIXES
] + [
    f"{GRIPPER_B_CONTROLLER_PREFIX}{suffix}" for suffix in INNER_GRIPPER_CONTROLLERS_SUFFIXES
]

ALL_GRIPPER_EXTERNAL_CONTROLLERS = [
    f"{GRIPPER_A_CONTROLLER_PREFIX}{suffix}" for suffix in EXTERNAL_GRIPPER_CONTROLLERS_SUFFIXES
] + [
    f"{GRIPPER_B_CONTROLLER_PREFIX}{suffix}" for suffix in EXTERNAL_GRIPPER_CONTROLLERS_SUFFIXES
]

# DH-Robotics AG-95 夹爪控制器后缀
AG95_CONTROLLER_SUFFIXES = [
    "gripper_controller",
]

ALL_AG95_CONTROLLERS = [
    f"{AG95_A_CONTROLLER_PREFIX}{suffix}" for suffix in AG95_CONTROLLER_SUFFIXES
] + [
    f"{AG95_B_CONTROLLER_PREFIX}{suffix}" for suffix in AG95_CONTROLLER_SUFFIXES
]


def _read_configured_gripper_type():
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
    ur_headless_mode = LaunchConfiguration("ur_headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    activate_ur_controller = LaunchConfiguration("activate_ur_controller")
    activate_ur_state_controller = LaunchConfiguration("activate_ur_state_controller")
    activate_gripper_controller = LaunchConfiguration("activate_gripper_controller")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    initial_ur_suffix = LaunchConfiguration("initial_ur_controller").perform(context)
    _gripper_type = _read_configured_gripper_type()
    HW = _load_hardware_params()

    # ### 激活的控制器列表 ###
    controllers_active = list(BASE_ACTIVE_CONTROLLERS)
    # 添加初始ur控制器
    ur_A_initial_controller = f"{UR_A_CONTROLLER_PREFIX}{initial_ur_suffix}"
    ur_B_initial_controller = f"{UR_B_CONTROLLER_PREFIX}{initial_ur_suffix}"
    if activate_ur_controller.perform(context).lower() == "true":
        controllers_active += [ 
            ur_A_initial_controller,
            ur_B_initial_controller,
        ]
    if activate_ur_state_controller.perform(context).lower() == "true" and use_fake_hardware.perform(context).lower() == "false":
        controllers_active += ALL_UR_A_STATE_CONTROLLERS
        controllers_active += ALL_UR_B_STATE_CONTROLLERS
    # 添加初始gripper控制器（根据 gripper_type 选择）
    if activate_gripper_controller.perform(context).lower() == "true":
        if _gripper_type == "robotiq_2f85":
            controllers_active += ALL_GRIPPER_INNER_CONTROLLERS
            if use_fake_hardware.perform(context).lower() == "false":
                controllers_active += ALL_GRIPPER_EXTERNAL_CONTROLLERS
        elif _gripper_type == "dh_ag95":
            controllers_active += ALL_AG95_CONTROLLERS

    # ### 未激活的控制器列表 ###
    controllers_inactive = [] 
    for controller_name in ALL_UR_A_CONTROLLERS + ALL_UR_B_CONTROLLERS:
        if controller_name not in controllers_active:
            controllers_inactive.append(controller_name)



    description_file = PathJoinSubstitution( # 机器人描述文件
        [
            FindPackageShare(CONTROL_PACKAGE),
            DESCRIPTION_FILE,
        ]
    )

    controllers_file = PathJoinSubstitution( # 控制器配置文件
        [
            FindPackageShare(CONTROL_PACKAGE),
            CONTROLLERS_FILE,
        ]
    )

    rviz_config_file = PathJoinSubstitution( # RVIZ 配置文件
        [
            FindPackageShare(CONTROL_PACKAGE),
            RVIZ_CONFIG_FILE,
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",

            # 经常修改的启动参数
            "use_fake_hardware:=", use_fake_hardware, " ",
            "ur_headless_mode:=", ur_headless_mode, " ",

            # 固定参数：UR 网络（来自 hardware_interface_controller_params.yaml）
            "ur_reverse_ip:=", str(HW["ur"]["reverse_ip"]), " ",
            "ur_A_robot_ip:=", str(HW["ur"]["arm_A"]["robot_ip"]), " ",
            "ur_B_robot_ip:=", str(HW["ur"]["arm_B"]["robot_ip"]), " ",

            # 固定参数：UR A 端口
            "ur_A_reverse_port:=", str(HW["ur"]["arm_A"]["reverse_port"]), " ",
            "ur_A_script_sender_port:=", str(HW["ur"]["arm_A"]["script_sender_port"]), " ",
            "ur_A_script_command_port:=", str(HW["ur"]["arm_A"]["script_command_port"]), " ",
            "ur_A_trajectory_port:=", str(HW["ur"]["arm_A"]["trajectory_port"]), " ",
            "ur_A_rw_rate:=", str(HW["ur"]["arm_A"]["rw_rate"]), " ",

            # 固定参数：UR B 端口
            "ur_B_reverse_port:=", str(HW["ur"]["arm_B"]["reverse_port"]), " ",
            "ur_B_script_sender_port:=", str(HW["ur"]["arm_B"]["script_sender_port"]), " ",
            "ur_B_script_command_port:=", str(HW["ur"]["arm_B"]["script_command_port"]), " ",
            "ur_B_trajectory_port:=", str(HW["ur"]["arm_B"]["trajectory_port"]), " ",
            "ur_B_rw_rate:=", str(HW["ur"]["arm_B"]["rw_rate"]), " ",

            # 固定参数：Robotiq 串口
            "g2f85_A_com_port:=", str(HW["g2f85"]["A_com_port"]), " ",
            "g2f85_B_com_port:=", str(HW["g2f85"]["B_com_port"]), " ",

            # 固定参数：DH-Robotics AG-95 共享
            "ag95_gripper_transport_type:=", str(HW["ag95"]["transport_type"]), " ",
            "ag95_gripper_serial_baudrate:=", str(HW["ag95"]["serial_baudrate"]), " ",
            "ag95_gripper_can_bitrate:=", str(HW["ag95"]["can_bitrate"]), " ",
            "ag95_gripper_pcan_bitrate:=", str(HW["ag95"]["pcan_bitrate"]), " ",
            "ag95_gripper_gripper_model:=", str(HW["ag95"]["gripper_model"]), " ",
            "ag95_gripper_default_force_percent:=", str(HW["ag95"]["default_force_percent"]), " ",
            "ag95_gripper_auto_initialize:=", str(HW["ag95"]["auto_initialize"]), " ",
            "ag95_gripper_rw_rate:=", str(HW["ag95"]["rw_rate"]), " ",
            "ag95_gripper_command_interval_ms:=", str(HW["ag95"]["command_interval_ms"]), " ",
            # 固定参数：DH-Robotics AG-95 臂A
            "ag95_A_serial_port:=", str(HW["ag95"]["arm_A"]["serial_port"]), " ",
            "ag95_A_can_interface:=", str(HW["ag95"]["arm_A"]["can_interface"]), " ",
            "ag95_A_pcan_channel:=", str(HW["ag95"]["arm_A"]["pcan_channel"]), " ",
            "ag95_A_gripper_id:=", str(HW["ag95"]["arm_A"]["gripper_id"]), " ",
            # 固定参数：DH-Robotics AG-95 臂B
            "ag95_B_serial_port:=", str(HW["ag95"]["arm_B"]["serial_port"]), " ",
            "ag95_B_can_interface:=", str(HW["ag95"]["arm_B"]["can_interface"]), " ",
            "ag95_B_pcan_channel:=", str(HW["ag95"]["arm_B"]["pcan_channel"]), " ",
            "ag95_B_gripper_id:=", str(HW["ag95"]["arm_B"]["gripper_id"]), " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 1. robot_state_publisher
    # Jazzy 中 robot_description 由 robot_state_publisher 发布，ros2_control_node 从 topic 获取。
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 2. 单一 controller_manager：同时加载双 UR + 双 Robotiq
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            ParameterFile(controllers_file, allow_substs=True),
        ],
    )

    # 3. 基础节点
    nodes = [
        robot_state_publisher_node,
        control_node,
    ]

    # 4. robot_state_helper：真实硬件时启动
    if LAUNCH_ROBOT_STATE_HELPER:
        arm_A_robot_state_helper = Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            namespace="arm_A",
            name="ur_robot_state_helper",
            output="screen",
            parameters=[
                {"headless_mode": ur_headless_mode},
                {"robot_ip": str(HW["ur"]["arm_A"]["robot_ip"])},
            ],
            condition=UnlessCondition(use_fake_hardware),
        )

        arm_B_robot_state_helper = Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            namespace="arm_B",
            name="ur_robot_state_helper",
            output="screen",
            parameters=[
                {"headless_mode": ur_headless_mode},
                {"robot_ip": str(HW["ur"]["arm_B"]["robot_ip"])},
            ],
            condition=UnlessCondition(use_fake_hardware),
        )

        nodes += [
            arm_A_robot_state_helper,
            arm_B_robot_state_helper,
        ]

    # 5. urscript_interface：真实硬件时启动
    if LAUNCH_URSCRIPT_INTERFACE:
        arm_A_urscript_interface = Node(
            package="ur_robot_driver",
            executable="urscript_interface",
            namespace="arm_A",
            name="urscript_interface",
            output="screen",
            parameters=[
                {"robot_ip": str(HW["ur"]["arm_A"]["robot_ip"])},
            ],
            condition=UnlessCondition(use_fake_hardware),
        )

        arm_B_urscript_interface = Node(
            package="ur_robot_driver",
            executable="urscript_interface",
            namespace="arm_B",
            name="urscript_interface",
            output="screen",
            parameters=[
                {"robot_ip": str(HW["ur"]["arm_B"]["robot_ip"])},
            ],
            condition=UnlessCondition(use_fake_hardware),
        )

        nodes += [
            arm_A_urscript_interface,
            arm_B_urscript_interface,
        ]

    # 6. controller_stopper：默认不启用
    if LAUNCH_CONTROLLER_STOPPER:
        arm_A_controller_stopper = Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            namespace="arm_A",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"headless_mode": ur_headless_mode},
                {"joint_controller_active": activate_ur_controller},
                {
                    "consistent_controllers": controllers_active,
                },
            ],
            condition=UnlessCondition(use_fake_hardware),
        )

        arm_B_controller_stopper = Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            namespace="arm_B",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"headless_mode": ur_headless_mode},
                {"joint_controller_active": activate_ur_controller},
                {
                    "consistent_controllers": controllers_active,
                },
            ],
            condition=UnlessCondition(use_fake_hardware),
        )

        nodes += [
            arm_A_controller_stopper,
            arm_B_controller_stopper,
        ]

    # 7. 控制器 spawner
    def controller_spawner(controller_names, active=True):
        inactive_flag = ["--inactive"] if not active else []

        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            parameters=[
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

    active_controller_spawner = controller_spawner(controllers_active, active=True)

    inactive_controller_spawner = None
    if controllers_inactive:
        inactive_controller_spawner = controller_spawner(controllers_inactive, active=False)

    nodes.append(active_controller_spawner)

    if inactive_controller_spawner is not None:
        nodes.append(inactive_controller_spawner)

    # 8. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use mock hardware for UR and Robotiq.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_headless_mode",
            default_value="false",
            description="Enable UR headless mode.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_ur_controller",
            default_value="scaled_joint_trajectory_controller",
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
            description="Activate the selected initial UR controllers.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_ur_state_controller",
            default_value="true",
            description="Activate the UR state controllers.",
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
            default_value="10",
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

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
