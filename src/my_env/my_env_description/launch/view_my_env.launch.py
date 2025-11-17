from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable

def generate_launch_description():
    description_package = "my_env_description"
    description_file = "my_env.urdf.xacro"
    rvizconfig_file = "my_env_description.rviz"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}
    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 带有可视化可交互的关节状态发布节点，可控制 rviz 中机器人的关节角度
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rvizconfig_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", rvizconfig_file])
    # 启动 rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rvizconfig_file],
    )

    return LaunchDescription(
        [joint_state_publisher_gui_node, robot_state_publisher_node, rviz_node]
    )
