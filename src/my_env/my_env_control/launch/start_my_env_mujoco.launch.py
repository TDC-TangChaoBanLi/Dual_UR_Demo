from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():

    launch_rviz = LaunchConfiguration('launch_rviz')

    arm_A_tf_prefix = LaunchConfiguration('arm_A_prefix')
    arm_B_tf_prefix = LaunchConfiguration('arm_B_prefix')

    mjcf_file_path = PathJoinSubstitution([FindPackageShare('my_env_mujoco'), 'mjcf', 'my_env_mujoco.xml'])
    controller_parameters = PathJoinSubstitution([FindPackageShare("my_env_control"), "config", "my_env_mujoco_controller.yaml"])
    rviz_file_path = PathJoinSubstitution([FindPackageShare('my_env_control'), 'rviz', 'my_env_mujoco.rviz'])


    launch_rviz_arg = DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    arm_A_tf_prefix_arg = DeclareLaunchArgument(
            "arm_A_prefix",
            default_value="arm_A_",
            description="Prefix for arm_A_.",
        )

    arm_B_tf_prefix_arg = DeclareLaunchArgument(
            "arm_B_prefix",
            default_value="arm_B_",
            description="Prefix for arm_B_.",
        )


    # robot_description: 假定你有一个 xacro 可以产出包含两臂的 model
    # 例如 my_robot.urdf.xacro 接受 arm_A_tf_prefix & arm_B_tf_prefix 做命名
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('my_env_mujoco'), 'urdf', 'my_env_mujoco.urdf.xacro']), ' ',
        'arm_A_tf_prefix:=', arm_A_tf_prefix, ' ',
        'arm_B_tf_prefix:=', arm_B_tf_prefix, ' ',
        'use_fake_hardware:=true', ' ',
        'fake_sensor_commands:=true', ' ',
        'use_sim_mujoco:=true', ' ',
        'mjcf_file_path:=', mjcf_file_path, ' ',
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
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controller_parameters, allow_substs=True),
        ],
        remappings=[("~/robot_description", "/robot_description")],
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
            "arm_A_joint_trajectory_controller",
            "arm_B_joint_trajectory_controller",
            "arm_A_robotiq_gripper_controller",
            "arm_B_robotiq_gripper_controller"
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
            arm_A_tf_prefix_arg,
            arm_B_tf_prefix_arg,
            robot_state_publisher_node,
            control_node,
            spawn_joint_state_broadcaster,
            spawn_position_controller,
            rviz_node,
        ]
    )
