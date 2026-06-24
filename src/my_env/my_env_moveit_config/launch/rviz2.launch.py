import os
import yaml

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


MOVEIT_CONFIG_PACKAGE = "my_env_moveit_config"
DESCRIPTION_PACKAGE = "my_env_description"

# 这里改成你的纯描述 xacro 文件名
DESCRIPTION_FILE = "urdf/my_env.urdf.xacro"

CAMERA_USE_NOMINAL_EXTRINSICS = "true"

SAFETY_LIMITS = "false"
SAFETY_POS_MARGIN = "0.15"
SAFETY_K_POSITION = "20"
FORCE_ABS_PATHS = "false"


def load_yaml(package_name, file_path):
    package_path = FindPackageShare(package_name).find(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def load_file(package_name, file_path):
    package_path = FindPackageShare(package_name).find(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r") as f:
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
    robot_description = generate_robot_description()

    robot_description_semantic = {
        "robot_description_semantic": load_file(
            MOVEIT_CONFIG_PACKAGE,
            "config/my_env.srdf",
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            MOVEIT_CONFIG_PACKAGE,
            "config/kinematics.yaml",
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            MOVEIT_CONFIG_PACKAGE,
            "config/joint_limits.yaml",
        )
    }

    moveit_controllers = load_yaml(
        MOVEIT_CONFIG_PACKAGE,
        "config/moveit_controllers.yaml",
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare(MOVEIT_CONFIG_PACKAGE),
            "config",
            "moveit.rviz",
        ]
    )

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
    )

    return LaunchDescription([rviz_node])