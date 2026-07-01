from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def _load_main_launch_config():
    main_launch_file = Path(__file__).with_name("start_my_env_control.launch.py")
    spec = spec_from_file_location("start_my_env_control_launch", main_launch_file)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load launch config from {main_launch_file}")

    main_launch_config = module_from_spec(spec)
    spec.loader.exec_module(main_launch_config)
    return main_launch_config


_MAIN_LAUNCH_CONFIG = _load_main_launch_config()
UR_A_ROBOT_IP = _MAIN_LAUNCH_CONFIG.UR_A_ROBOT_IP
UR_B_ROBOT_IP = _MAIN_LAUNCH_CONFIG.UR_B_ROBOT_IP


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
