from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


MOVEIT_CONFIG_PACKAGE = "lab_bench_moveit_config"
CONTROL_PACKAGE = "launch_controller"

# 改成你 launch_controller 包里真实的 launch 文件名
CONTROL_LAUNCH_FILE = "start_traditional_controllers.launch.py"


def generate_launch_description():
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(CONTROL_PACKAGE),
                    "launch",
                    CONTROL_LAUNCH_FILE,
                ]
            )
        ),
        launch_arguments={
            "use_fake_hardware": "true",
            "use_fake_sensor_commands": "true",
            "ur_headless_mode": "false",
            "activate_ur_controller": "true",
            "activate_ur_state_controller": "false",
            "activate_gripper_controller": "true",
            "launch_rviz": "false",
        }.items(),
    )

    move_group_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(MOVEIT_CONFIG_PACKAGE),
                            "launch",
                            "move_group.launch.py",
                        ]
                    )
                )
            )
        ],
    )

    rviz_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(MOVEIT_CONFIG_PACKAGE),
                            "launch",
                            "moveit_rviz.launch.py",
                        ]
                    )
                )
            )
        ],
    )

    return LaunchDescription(
        [
            control_launch,
            move_group_launch,
            rviz_launch,
        ]
    )