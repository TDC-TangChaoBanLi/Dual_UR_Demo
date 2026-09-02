from pathlib import Path
import os
import xml.etree.ElementTree as ET
import yaml

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
# 环境包命名规则：{env}_description
# 每个环境包内固定结构：
#   xacro/ros2_control/{env}.xacro          # ros2_control 入口
#   config/ros2_control/traditional_controllers.yaml
#   rviz/{env}_traditional.rviz
#   mujoco 环境额外：
#     config/mujoco_ros2_control_plugins.yaml
#     config/mujoco_pids_config.yaml
#     mjcf/{env}.xml
# ============================================================

SUPPORTED_GRIPPER_TYPES = {"dh_ag95", "robotiq_2f85", "none"}


# ============================================================
# 控制器固定名称：必须与 traditional_controllers.yaml 一致
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

# UR 传统（非 mujoco）可选控制器后缀
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

# UR mujoco 仿真可用运动控制器后缀
# [WARN] 必须和 controller.yaml / MoveIt 配置保持一致
MUJOCO_UR_CONTROLLER_SUFFIXES = [
    "joint_trajectory_controller",
    "forward_position_controller",
    "forward_velocity_controller",
    "forward_effort_controller",
]

# UR 传统状态控制器后缀（真实硬件）
ALL_UR_STATE_CONTROLLERS_SUFFIXES = [
    "io_and_status_controller",
    "speed_scaling_state_broadcaster",
    "force_torque_sensor_broadcaster",
    "tcp_pose_broadcaster",
    "ur_configuration_controller",
]

# UR mujoco 仿真可用状态控制器后缀
MUJOCO_UR_STATE_CONTROLLERS_SUFFIXES = [
    "force_torque_sensor_broadcaster",
]

# Robotiq 控制器前缀
GRIPPER_A_CONTROLLER_PREFIX = "arm_A_g2f85_"
GRIPPER_B_CONTROLLER_PREFIX = "arm_B_g2f85_"

# DH-Robotics AG-95 控制器前缀
AG95_A_CONTROLLER_PREFIX = "arm_A_dhag95_"
AG95_B_CONTROLLER_PREFIX = "arm_B_dhag95_"

# 双 Robotiq 控制器名称后缀
INNER_GRIPPER_CONTROLLERS_SUFFIXES = [
    "gripper_controller",
]
EXTERNAL_GRIPPER_CONTROLLERS_SUFFIXES = [
    "activation_controller",
]

# DH-Robotics AG-95 夹爪控制器后缀
AG95_CONTROLLER_SUFFIXES = [
    "gripper_controller",
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
    """读取统一硬件接口参数配置文件。

    硬件参数（UR 网络 / Robotiq 串口 / DH-AG95 夹爪）集中定义在
    {env}_description/config/ros2_control/hardware_interface_controller_params.yaml。
    """
    config_file = (
        Path(get_package_share_directory(_env_package(env)))
        / "config" / "ros2_control" / "hardware_interface_controller_params.yaml"
    )
    with open(config_file, "r") as f:
        return yaml.safe_load(f)


def _read_configured_gripper_type(env, is_mujoco):
    """从 dual_arm_config.xacro 读取夹爪类型。

    mujoco 环境复用对应非 mujoco 环境的几何描述
    （lab_bench_mujoco -> lab_bench_description）。
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


def _build_controller_lists(env, is_mujoco, use_fake_hardware, initial_ur_suffix):
    """根据环境类型构建激活/未激活控制器列表。"""
    if is_mujoco:
        ur_suffixes = MUJOCO_UR_CONTROLLER_SUFFIXES
        state_suffixes = MUJOCO_UR_STATE_CONTROLLERS_SUFFIXES
    else:
        ur_suffixes = ALL_UR_CONTROLLER_SUFFIXES
        state_suffixes = ALL_UR_STATE_CONTROLLERS_SUFFIXES

    all_ur_a = [f"{UR_A_CONTROLLER_PREFIX}{s}" for s in ur_suffixes]
    all_ur_b = [f"{UR_B_CONTROLLER_PREFIX}{s}" for s in ur_suffixes]
    all_ur_state_a = [f"{UR_A_CONTROLLER_PREFIX}{s}" for s in state_suffixes]
    all_ur_state_b = [f"{UR_B_CONTROLLER_PREFIX}{s}" for s in state_suffixes]

    controllers_active = list(BASE_ACTIVE_CONTROLLERS)

    # 初始 UR 运动控制器
    ur_A_initial_controller = f"{UR_A_CONTROLLER_PREFIX}{initial_ur_suffix}"
    ur_B_initial_controller = f"{UR_B_CONTROLLER_PREFIX}{initial_ur_suffix}"
    controllers_active += [ur_A_initial_controller, ur_B_initial_controller]

    # UR 状态控制器（传统模式仅真实硬件时激活；mujoco 模式始终可用）
    if is_mujoco:
        controllers_active += all_ur_state_a + all_ur_state_b
    elif use_fake_hardware.lower() == "false":
        controllers_active += all_ur_state_a + all_ur_state_b

    # 夹爪控制器（根据 gripper_type 选择）
    gripper_type = _read_configured_gripper_type(env, is_mujoco)
    if gripper_type == "robotiq_2f85":
        controllers_active += [
            f"{GRIPPER_A_CONTROLLER_PREFIX}{s}" for s in INNER_GRIPPER_CONTROLLERS_SUFFIXES
        ]
        controllers_active += [
            f"{GRIPPER_B_CONTROLLER_PREFIX}{s}" for s in INNER_GRIPPER_CONTROLLERS_SUFFIXES
        ]
        if not is_mujoco and use_fake_hardware.lower() == "false":
            controllers_active += [
                f"{GRIPPER_A_CONTROLLER_PREFIX}{s}" for s in EXTERNAL_GRIPPER_CONTROLLERS_SUFFIXES
            ]
            controllers_active += [
                f"{GRIPPER_B_CONTROLLER_PREFIX}{s}" for s in EXTERNAL_GRIPPER_CONTROLLERS_SUFFIXES
            ]
    elif gripper_type == "dh_ag95":
        controllers_active += [
            f"{AG95_A_CONTROLLER_PREFIX}{s}" for s in AG95_CONTROLLER_SUFFIXES
        ]
        controllers_active += [
            f"{AG95_B_CONTROLLER_PREFIX}{s}" for s in AG95_CONTROLLER_SUFFIXES
        ]
    elif gripper_type == "none":
        # 无夹爪环境（如 space_sim）：不添加任何夹爪控制器
        pass

    # 未激活的 UR 运动控制器
    controllers_inactive = []
    for controller_name in all_ur_a + all_ur_b:
        if controller_name not in controllers_active:
            controllers_inactive.append(controller_name)

    return controllers_active, controllers_inactive


def _build_xacro_args(env, is_mujoco, context):
    """构建 xacro 命令行参数列表。"""
    args = []

    if is_mujoco:
        share = get_package_share_directory(_env_package(env))
        mjcf_file = os.path.join(share, "mjcf", f"{env}.xml")
        pids_file = os.path.join(share, "config", "mujoco_pids_config.yaml")
        args += [
            "mjcf_file_path:=" + mjcf_file + " ",
            "pids_config_file_path:=" + pids_file + " ",
            "camera_use_nominal_extrinsics:=false ",
            "mujoco_sim_speed_factor:=" + LaunchConfiguration("mujoco_sim_speed_factor").perform(context) + " ",
            "mujoco_headless:=" + LaunchConfiguration("mujoco_headless").perform(context) + " ",
        ]
    else:
        use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
        ur_headless_mode = LaunchConfiguration("ur_headless_mode").perform(context)
        HW = _load_hardware_params(env)
        ur = HW["ur"]
        g2 = HW["g2f85"]
        ag = HW["ag95"]
        args += [
            "use_fake_hardware:=" + use_fake_hardware + " ",
            "ur_headless_mode:=" + ur_headless_mode + " ",
            "ur_reverse_ip:=" + ur["reverse_ip"] + " ",
            "ur_A_robot_ip:=" + ur["arm_A"]["robot_ip"] + " ",
            "ur_B_robot_ip:=" + ur["arm_B"]["robot_ip"] + " ",
            "ur_A_reverse_port:=" + ur["arm_A"]["reverse_port"] + " ",
            "ur_A_script_sender_port:=" + ur["arm_A"]["script_sender_port"] + " ",
            "ur_A_script_command_port:=" + ur["arm_A"]["script_command_port"] + " ",
            "ur_A_trajectory_port:=" + ur["arm_A"]["trajectory_port"] + " ",
            "ur_A_rw_rate:=" + ur["arm_A"]["rw_rate"] + " ",
            "ur_B_reverse_port:=" + ur["arm_B"]["reverse_port"] + " ",
            "ur_B_script_sender_port:=" + ur["arm_B"]["script_sender_port"] + " ",
            "ur_B_script_command_port:=" + ur["arm_B"]["script_command_port"] + " ",
            "ur_B_trajectory_port:=" + ur["arm_B"]["trajectory_port"] + " ",
            "ur_B_rw_rate:=" + ur["arm_B"]["rw_rate"] + " ",
            "g2f85_A_com_port:=" + g2["A_com_port"] + " ",
            "g2f85_B_com_port:=" + g2["B_com_port"] + " ",
            "ag95_gripper_transport_type:=" + ag["transport_type"] + " ",
            "ag95_gripper_serial_baudrate:=" + ag["serial_baudrate"] + " ",
            "ag95_gripper_can_bitrate:=" + ag["can_bitrate"] + " ",
            "ag95_gripper_pcan_bitrate:=" + ag["pcan_bitrate"] + " ",
            "ag95_gripper_gripper_model:=" + ag["gripper_model"] + " ",
            "ag95_gripper_default_force_percent:=" + ag["default_force_percent"] + " ",
            "ag95_gripper_auto_initialize:=" + ag["auto_initialize"] + " ",
            "ag95_gripper_rw_rate:=" + ag["rw_rate"] + " ",
            "ag95_gripper_command_interval_ms:=" + ag["command_interval_ms"] + " ",
            "ag95_A_serial_port:=" + ag["arm_A"]["serial_port"] + " ",
            "ag95_A_can_interface:=" + ag["arm_A"]["can_interface"] + " ",
            "ag95_A_pcan_channel:=" + ag["arm_A"]["pcan_channel"] + " ",
            "ag95_A_gripper_id:=" + ag["arm_A"]["gripper_id"] + " ",
            "ag95_B_serial_port:=" + ag["arm_B"]["serial_port"] + " ",
            "ag95_B_can_interface:=" + ag["arm_B"]["can_interface"] + " ",
            "ag95_B_pcan_channel:=" + ag["arm_B"]["pcan_channel"] + " ",
            "ag95_B_gripper_id:=" + ag["arm_B"]["gripper_id"] + " ",
        ]
    return args


def launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration("env").perform(context)
    is_mujoco = _is_mujoco_env(env)

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    initial_ur_suffix = LaunchConfiguration("initial_ur_controller").perform(context)

    # ### 激活/未激活控制器列表 ###
    controllers_active, controllers_inactive = _build_controller_lists(
        env, is_mujoco, use_fake_hardware.perform(context), initial_ur_suffix
    )

    # ### 文件路径（{env}_description 包内固定结构） ###
    description_file = PathJoinSubstitution(
        [
            FindPackageShare(_env_package(env)),
            "xacro", "ros2_control", f"{env}.xacro",
        ]
    )

    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare(_env_package(env)),
            "config", "ros2_control", "traditional_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(_env_package(env)),
            "rviz", f"{env}_traditional.rviz",
        ]
    )

    # ### robot_description ###
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
        ]
        + _build_xacro_args(env, is_mujoco, context)
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
        parameters=[robot_description]
        + ([{"use_sim_time": True}, {"publish_frequency": 100.0}] if is_mujoco else []),
    )

    # 2. ros2_control_node（mujoco 使用 mujoco_ros2_control 包）
    if is_mujoco:
        mujoco_plugins_file = PathJoinSubstitution(
            [
                FindPackageShare(_env_package(env)),
                "config", "mujoco_ros2_control_plugins.yaml",
            ]
        )
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
    else:
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                ParameterFile(controllers_file, allow_substs=True),
            ],
        )

    nodes = [
        robot_state_publisher_node,
        control_node,
    ]

    # 3. 控制器 spawner
    def controller_spawner(controller_names, active=True):
        inactive_flag = ["--inactive"] if not active else []

        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            parameters=[
                ({"use_sim_time": True} if is_mujoco else {}),
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
    nodes.append(active_controller_spawner)

    if controllers_inactive:
        inactive_controller_spawner = controller_spawner(controllers_inactive, active=False)
        nodes.append(inactive_controller_spawner)

    # 4. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}] if is_mujoco else [],
        condition=IfCondition(launch_rviz),
    )
    nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "env",
            default_value="lab_bench",
            description="Environment to launch controllers for (e.g. lab_bench, lab_bench_mujoco).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use mock hardware for UR and Robotiq (traditional env only).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_headless_mode",
            default_value="false",
            description="Enable UR headless mode (traditional env only).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_ur_controller",
            default_value="scaled_joint_trajectory_controller",
            description=(
                "Initial UR controller suffix. "
                "The launch file will expand it to arm_A_ur_* and arm_B_ur_*. "
                "For mujoco env, use e.g. forward_position_controller."
            ),
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "mujoco_headless",
            default_value="false",
            description="Run MuJoCo without the Simulate viewer window (mujoco env only).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mujoco_sim_speed_factor",
            default_value="1.0",
            description="Simulation speed factor (mujoco env only).",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )