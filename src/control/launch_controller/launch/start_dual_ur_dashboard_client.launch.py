import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _load_hardware_params():
    """从 lab_bench_description 读取统一硬件接口参数配置文件。"""
    config_file = os.path.join(
        get_package_share_directory("lab_bench_description"),
        "config", "ros2_control", "hardware_interface_controller_params.yaml",
    )
    with open(config_file, "r") as f:
        return yaml.safe_load(f)


_HW = _load_hardware_params()
UR_A_ROBOT_IP = _HW["ur"]["arm_A"]["robot_ip"]
UR_B_ROBOT_IP = _HW["ur"]["arm_B"]["robot_ip"]


def _dashboard_client_group(namespace, robot_ip, dashboard_receive_timeout):
    return GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_robot_driver"),
                            "launch",
                            "ur_dashboard_client.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "robot_ip": robot_ip,
                    "dashboard_receive_timeout": dashboard_receive_timeout,
                }.items(),
            ),
        ],
    )


def generate_launch_description():
    dashboard_receive_timeout_arg = DeclareLaunchArgument(
        "dashboard_receive_timeout",
        default_value="20.0",
        description="Timeout that each dashboard client will wait for a response from the robot.",
    )

    dashboard_receive_timeout = LaunchConfiguration("dashboard_receive_timeout")

    arm_A_dashboard_client = _dashboard_client_group(
        "arm_A",
        UR_A_ROBOT_IP,
        dashboard_receive_timeout,
    )

    arm_B_dashboard_client = _dashboard_client_group(
        "arm_B",
        UR_B_ROBOT_IP,
        dashboard_receive_timeout,
    )

    return LaunchDescription(
        [
            dashboard_receive_timeout_arg,
            arm_A_dashboard_client,
            arm_B_dashboard_client,
        ]
    )