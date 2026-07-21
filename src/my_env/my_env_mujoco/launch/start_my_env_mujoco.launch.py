import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():

    launch_rviz = LaunchConfiguration('launch_rviz')

    launch_rviz_arg = DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    xacro_file_path_arg = DeclareLaunchArgument(
            "xacro_file_path",
            default_value="urdf/my_env_mujoco.urdf.xacro",
            description="Relative path to xacro file in my_env_mujoco package.",
        )
    mjcf_file_path_arg = DeclareLaunchArgument(
            "mjcf_file_path",
            default_value="mjcf/my_env_mujoco.xml",
            description="Relative path to mjcf file in my_env_mujoco package.",
        )


    arm_A_tf_prefix = "arm_A_"
    arm_B_tf_prefix = "arm_B_"

    pkg_my_env_mujoco = FindPackageShare('my_env_mujoco')
    pkg_my_env_control = FindPackageShare('my_env_control')


    xacro_file_path = PathJoinSubstitution([pkg_my_env_mujoco, LaunchConfiguration('xacro_file_path')])
    mjcf_file_path = PathJoinSubstitution([pkg_my_env_mujoco, LaunchConfiguration('mjcf_file_path')])
    pids_config_file_path = PathJoinSubstitution([pkg_my_env_mujoco, 'config', 'mujoco_pids_config.yaml'])
    controller_parameters = PathJoinSubstitution([pkg_my_env_mujoco, "config", "my_env_mujoco_controller.yaml"])
    mujoco_plugins_file = PathJoinSubstitution([pkg_my_env_mujoco, "config", "mujoco_ros2_control_plugins.yaml"])
    rviz_file_path = PathJoinSubstitution([pkg_my_env_control, 'rviz', 'my_env_mujoco.rviz'])


    # robot_description: 假定你有一个 xacro 可以产出包含两臂的 model
    # 例如 my_robot.urdf.xacro 接受 arm_A_tf_prefix & arm_B_tf_prefix 做命名
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        xacro_file_path, ' ',
        'use_fake_hardware:=true', ' ',
        'fake_sensor_commands:=true', ' ',
        'use_sim_mujoco:=true', ' ',
        'mjcf_file_path:=', mjcf_file_path, ' ',
        'pids_config_file_path:=', pids_config_file_path, ' ',
        'generate_ros2_control:=false', ' ',
        'camera_use_nominal_extrinsics:=true',
    ])
    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controller_parameters, allow_substs=True),
            ParameterFile(mujoco_plugins_file)
        ],
        remappings=(
                [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
        ],
        output="both",
    )

    spawn_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_position_controller",
        arguments=[
            "arm_A_forward_position_controller",
            "arm_A_robotiq_gripper_controller",
            "arm_A_force_torque_sensor_broadcaster",

            "arm_B_forward_position_controller",
            "arm_B_robotiq_gripper_controller",
            "arm_B_force_torque_sensor_broadcaster",
        ],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file_path],
        parameters=[
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            launch_rviz_arg,
            xacro_file_path_arg,
            mjcf_file_path_arg,
            robot_state_publisher_node,
            control_node,
            spawn_joint_state_broadcaster,
            spawn_position_controller,
            rviz_node,
        ]
    )