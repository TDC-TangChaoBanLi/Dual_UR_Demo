import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
# Package configuration
# ============================================================

MOVEIT_CONFIG_PACKAGE = "my_env_moveit_config"

# 使用纯描述包，不使用 my_env_control 中带 ros2_control 的 URDF
DESCRIPTION_PACKAGE = "my_env_description"

# 改成你的纯描述文件实际路径
DESCRIPTION_FILE = "urdf/my_env.urdf.xacro"


# ============================================================
# Robot description parameters
# 这里只保留 MoveIt / RViz 真正需要的几何描述参数
# ============================================================

ARM_A_TF_PREFIX = "arm_A_" # [WARN] 不要改 Don't Change this
ARM_B_TF_PREFIX = "arm_B_" # [WARN] 不要改 Don't Change this

UR_A_TYPE = "ur5"
UR_B_TYPE = "ur5e"

CAMERA_USE_NOMINAL_EXTRINSICS = "true"

SAFETY_LIMITS = "false"
SAFETY_POS_MARGIN = "0.15"
SAFETY_K_POSITION = "20"
FORCE_ABS_PATHS = "false"


def package_file(package_name, *paths):
    return os.path.join(
        FindPackageShare(package_name).find(package_name),
        *paths,
    )


def load_yaml(package_name, *paths):
    file_path = package_file(package_name, *paths)
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def load_file(package_name, *paths):
    file_path = package_file(package_name, *paths)
    with open(file_path, "r") as f:
        return f.read()


def generate_robot_description():
    description_file = PathJoinSubstitution(
        [
            FindPackageShare(DESCRIPTION_PACKAGE),
            DESCRIPTION_FILE,
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",

            "camera_use_nominal_extrinsics:=", CAMERA_USE_NOMINAL_EXTRINSICS, " ",

            "safety_limits:=", SAFETY_LIMITS, " ",
            "safety_pos_margin:=", SAFETY_POS_MARGIN, " ",
            "safety_k_position:=", SAFETY_K_POSITION, " ",
            "force_abs_paths:=", FORCE_ABS_PATHS, " ",
        ]
    )

    return {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Whether to launch RViz with MoveIt MotionPlanning plugin.",
        )
    )

    # ============================================================
    # Common MoveIt parameters
    # ============================================================

    robot_description = generate_robot_description()

    robot_description_semantic = {
        "robot_description_semantic": load_file(
            MOVEIT_CONFIG_PACKAGE,
            "config",
            "my_env.srdf",
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            MOVEIT_CONFIG_PACKAGE,
            "config",
            "kinematics.yaml",
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            MOVEIT_CONFIG_PACKAGE,
            "config",
            "joint_limits.yaml",
        )
    }

    planning_pipelines_yaml = PathJoinSubstitution(
        [
            FindPackageShare(MOVEIT_CONFIG_PACKAGE),
            "config",
            "planning_pipelines.yaml",
        ]
    )

    move_group_common_yaml = PathJoinSubstitution(
        [
            FindPackageShare(MOVEIT_CONFIG_PACKAGE),
            "config",
            "move_group_common.yaml",
        ]
    )

    moveit_controllers = load_yaml(
        MOVEIT_CONFIG_PACKAGE,
        "config",
        "moveit_controllers_SimpleControllerManager.yaml",
        # "moveit_controllers_Ros2ControlManager.yaml",
        )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare(MOVEIT_CONFIG_PACKAGE),
            "config",
            "moveit.rviz",
        ]
    )

    # ============================================================
    # move_group
    # ============================================================

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines_yaml,
            move_group_common_yaml,
            moveit_controllers,
        ],
    )

    # ============================================================
    # RViz
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
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            move_group_node,
            rviz_node,
        ]
    )