import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    my_description_package = "my_env_control"
    my_description_file = "my_env_controlled.urdf.xacro"
    my_ur_controller_launch_file = "my_ur_control.launch.py"

    ur_A_namespace = "ur_A"
    ur_B_namespace = "ur_B"

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')    
    use_fake_sensor_commands = LaunchConfiguration('use_fake_sensor_commands')
    launch_rviz = LaunchConfiguration('launch_rviz')

    ur_headless_mode = LaunchConfiguration('ur_headless_mode')
    ur_initial_joint_controller = LaunchConfiguration('ur_initial_joint_controller')
    ur_reverse_ip = LaunchConfiguration('ur_reverse_ip')
    ur_runtime_config_package = LaunchConfiguration('ur_runtime_config_package')
    ur_controller_file = LaunchConfiguration('ur_controller_file')

    arm_A_tf_prefix = "arm_A_"
    ur_A_ur_type = "ur5e"
    ur_A_robot_ip = "192.168.1.101"
    ur_A_script_command_port = 50004
    ur_A_reverse_port = 50001
    ur_A_script_sender_port = 50002
    ur_A_trajectory_port = 50003

    arm_B_tf_prefix = "arm_B_"
    ur_B_ur_type = "ur5e"
    ur_B_robot_ip = "192.168.1.102"
    ur_B_script_command_port = 50004
    ur_B_reverse_port = 50001
    ur_B_script_sender_port = 50002
    ur_B_trajectory_port = 50003


    use_fake_hardware_arg = DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware instead of hardware components.",
        )
    use_fake_sensor_commands_arg = DeclareLaunchArgument(
            "use_fake_sensor_commands",
            default_value="true",
            description="Use fake hardware instead of hardware components.",
        )
    launch_rviz_arg = DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    
    ur_headless_mode_arg = DeclareLaunchArgument(
            "ur_headless_mode",
            default_value="false",
            description="Use fake hardware instead of hardware components.",
        )
    ur_initial_joint_controller_arg = DeclareLaunchArgument(
            "ur_initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="The initial controller to be activated on robot start.",
        )
    ur_reverse_ip_arg = DeclareLaunchArgument(
            "ur_reverse_ip",
            default_value="192.168.8.195",
            description="The IP address of the robot controller.",
        )

    ur_runtime_config_package_arg = DeclareLaunchArgument(
            "ur_runtime_config_package",
            default_value="my_env_control",
    )
    ur_controller_file_arg = DeclareLaunchArgument(
            "ur_controller_file",
            default_value="my_ur_controllers.yaml",
    )




    robot_description_content = ParameterValue(Command
    (
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare(my_description_package), "urdf", my_description_file]), " ",
            "arm_A_tf_prefix:=", arm_A_tf_prefix, " ",
            "arm_B_tf_prefix:=", arm_B_tf_prefix, " ",
            "use_fake_hardware:=", use_fake_hardware, " ",
            "use_fake_sensor_commands:=", use_fake_sensor_commands, " ",
            "use_sim_gazebo:=", "false", " ",
            "generate_ros2_control:=", "false", " ",
            "ur_reverse_ip:=", ur_reverse_ip, " ",
            "ur_headless_mode:=", ur_headless_mode, " ",
            "ur_A_ur_type:=", ur_A_ur_type, " ",
            "ur_B_ur_type:=", ur_B_ur_type, " ",
            "ur_A_robot_ip:=", ur_A_robot_ip, " ",
            "ur_B_robot_ip:=", ur_B_robot_ip, " ",
            "ur_A_reverse_port:=", ur_A_reverse_port, " ",
            "ur_A_script_sender_port:=", ur_A_script_sender_port, " ",
            "ur_A_script_command_port:=", ur_A_script_command_port, " ",
            "ur_A_trajectory_port:=", ur_A_trajectory_port, " ",
            "ur_B_reverse_port:=", ur_B_reverse_port, " ",
            "ur_B_script_sender_port:=", ur_B_script_sender_port, " ",
            "ur_B_script_command_port:=", ur_B_script_command_port, " ",
            "ur_B_trajectory_port:=", ur_B_trajectory_port, " ",
        ]
    ),
    value_type=str
    )
    robot_description = {"robot_description": robot_description_content}
    ur_robot_driver_path = get_package_share_directory(my_description_package)


    ur_A_tf_prefix = arm_A_tf_prefix+"ur_"
    ur_A = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_robot_driver_path, 'launch', my_ur_controller_launch_file)),
        launch_arguments={
            'ur_type': ur_A_ur_type,
            'use_tool_communication': 'false',
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': use_fake_sensor_commands,
            'headless_mode':ur_headless_mode,
            'initial_joint_controller':ur_initial_joint_controller,
            'reverse_ip': ur_reverse_ip,
            'robot_ip': ur_A_robot_ip,
            'tf_prefix': ur_A_tf_prefix,
            'script_command_port': ur_A_script_command_port,
            'trajectory_port': ur_A_trajectory_port,
            'reverse_port': ur_A_reverse_port,
            'script_sender_port': ur_A_script_sender_port,

            'runtime_config_package': ur_runtime_config_package,
            'controllers_file': ur_controller_file,
            # 'description_package': my_description_package,
            # 'description_file': my_description_file,
        }.items())
    
    ur_A_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(ur_A_namespace),
            ur_A,
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[robot_description]
            )         
        ]
    )


    ur_B_tf_prefix = arm_B_tf_prefix+"ur_"
    ur_B = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_robot_driver_path, 'launch', my_ur_controller_launch_file)),
        launch_arguments={
            'ur_type': ur_B_ur_type,
            'use_tool_communication': 'false',
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': use_fake_sensor_commands,
            'headless_mode':ur_headless_mode,
            'initial_joint_controller':ur_initial_joint_controller,
            'reverse_ip': ur_reverse_ip,
            'robot_ip': ur_B_robot_ip,
            'tf_prefix': ur_B_tf_prefix,
            'script_command_port': ur_B_script_command_port,
            'trajectory_port': ur_B_trajectory_port,
            'reverse_port': ur_B_reverse_port,
            'script_sender_port': ur_B_script_sender_port,

            'runtime_config_package': ur_runtime_config_package,
            'controllers_file': ur_controller_file,
            # 'description_package': my_description_package,
            # 'description_file': my_description_file,
        }.items())
    
    ur_B_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(ur_B_namespace),
            ur_B,
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[robot_description]
            )         
        ]
    )



    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_env_control"), "rviz", "urdf.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    

    return LaunchDescription([    
        use_fake_hardware_arg,
        use_fake_sensor_commands_arg,
        launch_rviz_arg,

        ur_headless_mode_arg,
        ur_initial_joint_controller_arg,
        ur_reverse_ip_arg,
        
        ur_runtime_config_package_arg,
        ur_controller_file_arg,

        rviz_node,# if you don't want to launch the rviz2 to show the robot state, comment it
        ur_A_with_namespace,
        ur_B_with_namespace
    ])