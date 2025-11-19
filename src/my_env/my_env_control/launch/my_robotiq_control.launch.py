from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

def generate_launch_description():
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_package = LaunchConfiguration("description_package")
    controllers_file = LaunchConfiguration("controllers_file")
    update_rate_file = LaunchConfiguration("update_rate_file")
    description_file = LaunchConfiguration("description_file")

    name = "robotiq_85"
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    com_port = LaunchConfiguration("com_port")
    gripper_speed_multiplier = "1.0"
    gripper_force_multiplier = "0.5"
    gripper_max_speed = "0.150"
    gripper_max_force = "235.0"
    gripper_closed_position = "0.7929"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="runtime_config_package",
            default_value="robotiq_description",
            description="Package with robot runtime configuration",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="description_package",
            default_value="my_env_control", # ! use the changed package name
            description="Package with robot description",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="controllers_file",
            default_value="robotiq_controllers.yaml",
            description="YAML file with the controllers configuration",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="update_rate_file",
            default_value="robotiq_update_rate.yaml",
            description="YAML file with the update rate configuration",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="description_file",
            default_value="robotiq_2f_85_gripper_control.urdf.xacro", # ! use the changed file name
            description="URDF/XACRO description file with robot hardware configuration",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="com_port",
            default_value="/dev/ttyUSB0",
            description="Port for communicating with Robotiq hardware",
        )
    )
    

    declared_arguments.append(
        DeclareLaunchArgument(
            name="tf_prefix",
            default_value="",
            description="Prefix for tf frames",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_fake_hardware",
            default_value="false",
            description="Whether to use fake hardware simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="mock_sensor_commands",
            default_value="false",
            description="Whether to simulate sensor commands",
        )
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
            "name:=", name, " ",
            "tf_prefix:=", tf_prefix, " ",
            "use_fake_hardware:=", use_fake_hardware, " ",
            "mock_sensor_commands:=", mock_sensor_commands, " ",
            "com_port:=", com_port, " ",
            "gripper_speed_multiplier:=", gripper_speed_multiplier, " ",
            "gripper_force_multiplier:=", gripper_force_multiplier, " ",
            "gripper_max_speed:=", gripper_max_speed, " ",
            "gripper_max_force:=", gripper_max_force, " ",
            "gripper_closed_position:=", gripper_closed_position, " ",
        ]
    )

    robot_description_param = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


    update_rate_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package),"config",update_rate_file]
        )
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True), # ! must be used allow_substs=True to substitute parameters in the yaml file
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager", #"/controller_manager", # ! change to relative path 
        ],
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "controller_manager"], #"/controller_manager", # ! change to relative path 
    )

    robotiq_activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_activation_controller", "-c", "controller_manager"], #"/controller_manager", # ! change to relative path 
    )

    nodes = [
        control_node,
        # robot_state_publisher_node,   # ! launch in external launch file
        joint_state_broadcaster_spawner,
        robotiq_gripper_controller_spawner,
        robotiq_activation_controller_spawner,
        # rviz_node,    # ! launch in external launch file
    ]

    return LaunchDescription(declared_arguments + nodes)
