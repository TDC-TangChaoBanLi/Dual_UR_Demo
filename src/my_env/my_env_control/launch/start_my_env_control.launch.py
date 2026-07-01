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

DESCRIPTION_FILE = "urdf/my_env_control.urdf.xacro"
CONTROLLERS_FILE = "config/dual_arm_controllers.yaml"
UPDATE_RATE_FILE = "config/dual_arm_update_rate.yaml"
RVIZ_CONFIG_FILE = "rviz/my_env_control.rviz"
# UR 运动学校准参数文件 my_env_control 包内
UR_A_KINEMATICS_PARAMETERS_FILE = "config/ur_A_kinematics_calibration.yaml"
UR_B_KINEMATICS_PARAMETERS_FILE = "config/ur_B_kinematics_calibration.yaml"


# 真实硬件 IP
UR_REVERSE_IP = "192.168.1.100"
UR_A_ROBOT_IP = "192.168.1.17"
UR_B_ROBOT_IP = "192.168.1.11"

# 两台 UR 的端口必须不同
UR_A_REVERSE_PORT = "50001"
UR_A_SCRIPT_SENDER_PORT = "50002"
UR_A_SCRIPT_COMMAND_PORT = "50003"
UR_A_TRAJECTORY_PORT = "50004"

UR_B_REVERSE_PORT = "50011"
UR_B_SCRIPT_SENDER_PORT = "50012"
UR_B_SCRIPT_COMMAND_PORT = "50013"
UR_B_TRAJECTORY_PORT = "50014"

# UR5 若为 CB3，建议 125；UR5e 可 500。
# 如果你想先保守调试，也可以把二者都设为 0，并在 update_rate yaml 中统一用 125。
UR_A_RW_RATE = "125"
UR_B_RW_RATE = "500"

# Robotiq 串口
G2F85_A_COM_PORT = "/dev/ttyUSB0"
G2F85_B_COM_PORT = "/dev/ttyUSB1"

# DH-Robotics AG-95 夹爪 共享参数
AG95_GRIPPER_TRANSPORT_TYPE = "socketcan"
AG95_GRIPPER_SERIAL_BAUDRATE = "115200"
AG95_GRIPPER_CAN_BITRATE = "500000"
AG95_GRIPPER_PCAN_BITRATE = "500000"
AG95_GRIPPER_GRIPPER_MODEL = "ag-160-95"
AG95_GRIPPER_DEFAULT_FORCE_PERCENT = "100"
AG95_GRIPPER_AUTO_INITIALIZE = "true"
AG95_GRIPPER_RW_RATE = "25"
AG95_GRIPPER_COMMAND_INTERVAL_MS = "10"
# DH-Robotics AG-95 臂A 独立参数
AG95_A_SERIAL_PORT = "/dev/ttyAG95_A"
AG95_A_CAN_INTERFACE = "can0"
AG95_A_PCAN_CHANNEL = "PCAN_USBBUS1"
AG95_A_GRIPPER_ID = "1"
# DH-Robotics AG-95 臂B 独立参数
AG95_B_SERIAL_PORT = "/dev/ttyAG95_B"
AG95_B_CAN_INTERFACE = "can0"
AG95_B_PCAN_CHANNEL = "PCAN_USBBUS1"
AG95_B_GRIPPER_ID = "2"

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
AG95_A_CONTROLLER_PREFIX = "arm_A_ag95_"
AG95_B_CONTROLLER_PREFIX = "arm_B_ag95_"

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



def launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_fake_sensor_commands = LaunchConfiguration("use_fake_sensor_commands")
    ur_headless_mode = LaunchConfiguration("ur_headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    activate_ur_controller = LaunchConfiguration("activate_ur_controller")
    activate_ur_state_controller = LaunchConfiguration("activate_ur_state_controller")
    activate_gripper_controller = LaunchConfiguration("activate_gripper_controller")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    gripper_type = LaunchConfiguration("gripper_type")

    initial_ur_suffix = LaunchConfiguration("initial_ur_controller").perform(context)
    _gripper_type = gripper_type.perform(context)

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

    update_rate_config_file = PathJoinSubstitution( # 控制更新率配置文件
        [
            FindPackageShare(CONTROL_PACKAGE),
            UPDATE_RATE_FILE,
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
            "use_fake_sensor_commands:=", use_fake_sensor_commands, " ",
            "ur_headless_mode:=", ur_headless_mode, " ",

            # 固定参数：UR 网络
            "ur_reverse_ip:=", UR_REVERSE_IP, " ",
            "ur_A_robot_ip:=", UR_A_ROBOT_IP, " ",
            "ur_B_robot_ip:=", UR_B_ROBOT_IP, " ",

            # 固定参数：UR A 端口
            "ur_A_reverse_port:=", UR_A_REVERSE_PORT, " ",
            "ur_A_script_sender_port:=", UR_A_SCRIPT_SENDER_PORT, " ",
            "ur_A_script_command_port:=", UR_A_SCRIPT_COMMAND_PORT, " ",
            "ur_A_trajectory_port:=", UR_A_TRAJECTORY_PORT, " ",
            "ur_A_rw_rate:=", UR_A_RW_RATE, " ",

            # 固定参数：UR B 端口
            "ur_B_reverse_port:=", UR_B_REVERSE_PORT, " ",
            "ur_B_script_sender_port:=", UR_B_SCRIPT_SENDER_PORT, " ",
            "ur_B_script_command_port:=", UR_B_SCRIPT_COMMAND_PORT, " ",
            "ur_B_trajectory_port:=", UR_B_TRAJECTORY_PORT, " ",
            "ur_B_rw_rate:=", UR_B_RW_RATE, " ",

            # 固定参数：Robotiq 串口
            "g2f85_A_com_port:=", G2F85_A_COM_PORT, " ",
            "g2f85_B_com_port:=", G2F85_B_COM_PORT, " ",

            # 固定参数：DH-Robotics AG-95 共享
            "ag95_gripper_transport_type:=", AG95_GRIPPER_TRANSPORT_TYPE, " ",
            "ag95_gripper_serial_baudrate:=", AG95_GRIPPER_SERIAL_BAUDRATE, " ",
            "ag95_gripper_can_bitrate:=", AG95_GRIPPER_CAN_BITRATE, " ",
            "ag95_gripper_pcan_bitrate:=", AG95_GRIPPER_PCAN_BITRATE, " ",
            "ag95_gripper_gripper_model:=", AG95_GRIPPER_GRIPPER_MODEL, " ",
            "ag95_gripper_default_force_percent:=", AG95_GRIPPER_DEFAULT_FORCE_PERCENT, " ",
            "ag95_gripper_auto_initialize:=", AG95_GRIPPER_AUTO_INITIALIZE, " ",
            "ag95_gripper_rw_rate:=", AG95_GRIPPER_RW_RATE, " ",
            "ag95_gripper_command_interval_ms:=", AG95_GRIPPER_COMMAND_INTERVAL_MS, " ",
            # 固定参数：DH-Robotics AG-95 臂A
            "ag95_A_serial_port:=", AG95_A_SERIAL_PORT, " ",
            "ag95_A_can_interface:=", AG95_A_CAN_INTERFACE, " ",
            "ag95_A_pcan_channel:=", AG95_A_PCAN_CHANNEL, " ",
            "ag95_A_gripper_id:=", AG95_A_GRIPPER_ID, " ",
            # 固定参数：DH-Robotics AG-95 臂B
            "ag95_B_serial_port:=", AG95_B_SERIAL_PORT, " ",
            "ag95_B_can_interface:=", AG95_B_CAN_INTERFACE, " ",
            "ag95_B_pcan_channel:=", AG95_B_PCAN_CHANNEL, " ",
            "ag95_B_gripper_id:=", AG95_B_GRIPPER_ID, " ",
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
            update_rate_config_file,
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
                {"robot_ip": UR_A_ROBOT_IP},
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
                {"robot_ip": UR_B_ROBOT_IP},
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
                {"robot_ip": UR_A_ROBOT_IP},
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
                {"robot_ip": UR_B_ROBOT_IP},
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
            "use_fake_sensor_commands",
            default_value="true",
            description="Enable fake sensor command interfaces when using mock hardware.",
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
            "gripper_type",
            default_value="dh_ag95",
            choices=["dh_ag95", "robotiq_2f85"],
            description="Gripper type: dh_ag95 or robotiq_2f85.",
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