"""只启动 RViz（move_group 已在别处运行时用）。

为了保持 launch 文件自包含（不依赖兄弟模块的导入路径），这里重复了少量加载 helper。
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MOVEIT_CONFIG_PACKAGE = "space_sim_moveit_config"
DESCRIPTION_PACKAGE = "space_sim_description"
DESCRIPTION_FILE = "xacro/space_sim.xacro"


def package_file(package_name, *paths):
    return os.path.join(
        FindPackageShare(package_name).find(package_name),
        *paths,
    )


def load_yaml(package_name, *paths):
    with open(package_file(package_name, *paths), "r") as f:
        return yaml.safe_load(f)


def load_file(package_name, *paths):
    with open(package_file(package_name, *paths), "r") as f:
        return f.read()


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz config file inside the package's config directory.",
        ),
    ]

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare(DESCRIPTION_PACKAGE), DESCRIPTION_FILE]
                    ),
                    " camera_use_nominal_extrinsics:=true safety_limits:=false",
                    " safety_pos_margin:=0.15 safety_k_position:=20 force_abs_paths:=false",
                ]
            ),
            value_type=str,
        )
    }

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare(MOVEIT_CONFIG_PACKAGE),
            "config",
            LaunchConfiguration("rviz_config"),
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
            {
                "robot_description_semantic": load_file(
                    MOVEIT_CONFIG_PACKAGE, "config", "space_sim.srdf"
                )
            },
            {
                "robot_description_kinematics": load_yaml(
                    MOVEIT_CONFIG_PACKAGE, "config", "kinematics.yaml"
                )
            },
            {
                "robot_description_planning": load_yaml(
                    MOVEIT_CONFIG_PACKAGE, "config", "joint_limits.yaml"
                )
            },
        ],
    )

    return LaunchDescription(declared_arguments + [rviz_node])
