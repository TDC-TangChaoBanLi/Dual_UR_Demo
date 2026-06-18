import os
import copy
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


# ============================================================
# Package constants
# ============================================================

MOVEIT_CONFIG_PACKAGE = "my_env_moveit_config"
DESCRIPTION_PACKAGE = "my_env_description"

# ============================================================
# File path constants
# ============================================================

DESCRIPTION_FILE = "urdf/my_env.urdf.xacro"

JOINT_LIMITS_FILE = "config/joint_limits.yaml"
KINEMATICS_FILE = "config/kinematics.yaml"
SRDF_FILE = "config/my_env.srdf"

SERVO_ARM_A_FILE = "config/servo_arm_A.yaml"
SERVO_ARM_B_FILE = "config/servo_arm_B.yaml"

# ============================================================
# Xacro argument constants
# ============================================================

ARM_A_TF_PREFIX = "arm_A_" # [WARN] Do not change this. 别改这个
ARM_B_TF_PREFIX = "arm_B_" # [WARN] Do not change this. 别改这个

UR_A_UR_TYPE = "ur5"
UR_B_UR_TYPE = "ur5e"

CAMERA_USE_NOMINAL_EXTRINSICS = "true"

SAFETY_LIMITS = "false"
SAFETY_POS_MARGIN = "0.15"
SAFETY_K_POSITION = "20"

FORCE_ABS_PATHS = "false"

# ============================================================
# Servo node constants
# ============================================================

ARM_A_NODE_NAME_PREFIX = "arm_A_"
ARM_B_NODE_NAME_PREFIX = "arm_B_"

SERVO_NODE_NAME_SUFFIX = "servo_node"

JOINT_STATE_TOPIC = "/joint_states"


def load_file(package_name: str, relative_file_path: str) -> str:
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, relative_file_path)

    if not os.path.exists(absolute_file_path):
        raise RuntimeError(f"File does not exist: {absolute_file_path}")

    with open(absolute_file_path, "r") as file:
        return file.read()


def load_yaml(package_name: str, relative_file_path: str) -> dict:
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, relative_file_path)

    if not os.path.exists(absolute_file_path):
        raise RuntimeError(f"File does not exist: {absolute_file_path}")

    with open(absolute_file_path, "r") as file:
        yaml_content = yaml.safe_load(file)

    if yaml_content is None:
        yaml_content = {}

    return yaml_content


def generate_robot_description():
    description_file = PathJoinSubstitution(
        [
            FindPackageShare(DESCRIPTION_PACKAGE),
            DESCRIPTION_FILE,
        ]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " ",
            f"arm_A_tf_prefix:={ARM_A_TF_PREFIX} ",
            f"arm_B_tf_prefix:={ARM_B_TF_PREFIX} ",
            f"ur_A_ur_type:={UR_A_UR_TYPE} ",
            f"ur_B_ur_type:={UR_B_UR_TYPE} ",
            f"camera_use_nominal_extrinsics:={CAMERA_USE_NOMINAL_EXTRINSICS} ",
            f"safety_limits:={SAFETY_LIMITS} ",
            f"safety_pos_margin:={SAFETY_POS_MARGIN} ",
            f"safety_k_position:={SAFETY_K_POSITION} ",
            f"force_abs_paths:={FORCE_ABS_PATHS} ",
        ]
    )

    return {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }


def generate_robot_description_semantic():
    robot_description_semantic_content = load_file(
        MOVEIT_CONFIG_PACKAGE,
        SRDF_FILE,
    )

    return {
        "robot_description_semantic": robot_description_semantic_content
    }


def generate_robot_description_kinematics():
    kinematics_yaml = load_yaml(
        MOVEIT_CONFIG_PACKAGE,
        KINEMATICS_FILE,
    )

    return {
        "robot_description_kinematics": kinematics_yaml
    }


def generate_robot_description_planning():
    joint_limits_yaml = load_yaml(
        MOVEIT_CONFIG_PACKAGE,
        JOINT_LIMITS_FILE,
    )

    return {
        "robot_description_planning": joint_limits_yaml
    }


def generate_planning_scene_monitor_parameters(arm_node_name_prefix: str):
    return {
        "planning_scene_monitor_options": {
            "name": f"{arm_node_name_prefix}_planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": JOINT_STATE_TOPIC,
            "attached_collision_object_topic": "attached_collision_object",
            "publish_planning_scene_topic": "planning_scene",
            "monitored_planning_scene_topic": "monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        }
    }


def generate_servo_parameters(servo_config_file: str):
    servo_yaml = load_yaml(
        MOVEIT_CONFIG_PACKAGE,
        servo_config_file,
    )

    # ------------------------------------------------------------
    # 当前启动文件按“不启动 move_group”的方式运行。
    #
    # 因此 Servo 自己作为 primary planning scene monitor。
    # 如果以后你重新改成和 move_group 同时运行，可以在 YAML 中改回 false，
    # 并删除这里的强制覆盖。
    # ------------------------------------------------------------
    servo_yaml["is_primary_planning_scene_monitor"] = True

    # 避免两个 Servo 节点同时启动时共用全局 /planning_scene 造成混乱。
    # 这里使用相对 topic，最终会被解析到：
    # /arm_A/monitored_planning_scene
    # /arm_B/monitored_planning_scene
    servo_yaml["monitored_planning_scene_topic"] = "monitored_planning_scene"

    # 双臂共用 joint_states。
    servo_yaml["joint_topic"] = JOINT_STATE_TOPIC

    return {
        "moveit_servo": servo_yaml
    }


def generate_servo_node(
    arm_node_name_prefix: str,
    servo_config_file: str,
    use_sim_time,
    robot_description,
    robot_description_semantic,
    robot_description_kinematics,
    robot_description_planning,
):
    servo_parameters = generate_servo_parameters(servo_config_file)

    planning_scene_monitor_parameters = generate_planning_scene_monitor_parameters(
        arm_node_name_prefix
    )

    return Node(
        package="moveit_servo",
        executable="servo_node",
        # namespace=arm_namespace,
        name=arm_node_name_prefix+SERVO_NODE_NAME_SUFFIX,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_scene_monitor_parameters,
            servo_parameters,
        ],
    )


def launch_setup(context, *args, **kwargs):
    servo_target = LaunchConfiguration("servo_target").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    servo_target = servo_target.strip()

    if servo_target not in ["A", "B", "both"]:
        raise RuntimeError(
            "Invalid servo_target. Valid values are: A, B, both"
        )

    robot_description = generate_robot_description()
    robot_description_semantic = generate_robot_description_semantic()
    robot_description_kinematics = generate_robot_description_kinematics()
    robot_description_planning = generate_robot_description_planning()

    nodes = []

    if servo_target in ["A", "both"]:
        nodes.append(
            generate_servo_node(
                arm_node_name_prefix=ARM_A_NODE_NAME_PREFIX,
                servo_config_file=SERVO_ARM_A_FILE,
                use_sim_time=use_sim_time,
                robot_description=copy.deepcopy(robot_description),
                robot_description_semantic=copy.deepcopy(robot_description_semantic),
                robot_description_kinematics=copy.deepcopy(robot_description_kinematics),
                robot_description_planning=copy.deepcopy(robot_description_planning),
            )
        )

    if servo_target in ["B", "both"]:
        nodes.append(
            generate_servo_node(
                arm_node_name_prefix=ARM_B_NODE_NAME_PREFIX,
                servo_config_file=SERVO_ARM_B_FILE,
                use_sim_time=use_sim_time,
                robot_description=copy.deepcopy(robot_description),
                robot_description_semantic=copy.deepcopy(robot_description_semantic),
                robot_description_kinematics=copy.deepcopy(robot_description_kinematics),
                robot_description_planning=copy.deepcopy(robot_description_planning),
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo_target",
                default_value="both",
                description="Which Servo node to launch. Valid values: A, B, both",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )