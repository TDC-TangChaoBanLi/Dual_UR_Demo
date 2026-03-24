from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    moveit_config_package = 'my_env_moveit_config'
    target_pose_topic = '/target_pose'
    current_pose_topic = '/current_pose' # TODO: 暂时不使用
    robot_description_topic = "robot_description"

    robot_description_semantic_path = os.path.join(get_package_share_directory(moveit_config_package), 'config', 'my_env.srdf')
    robot_description_semantic_content = open(robot_description_semantic_path, 'r').read()
    robot_description_kinematics_path = os.path.join(get_package_share_directory(moveit_config_package), 'config', 'kinematics.yaml')
    # robot_description_kinematics_content = yaml.safe_load(open(robot_description_kinematics_path, 'r'))

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            value=robot_description_semantic_content,
            value_type=str
        )
    }
    # robot_description_kinematics = {
    #     "robot_description_kinematics": robot_description_kinematics_content
    # }



    return LaunchDescription([
        # Node(
        #     package='joint_control',
        #     executable='mpc_ctrl',
        #     name='mpc_ctrl',
        #     output='screen'
        # ),
        Node(
            package='joint_control',
            executable='ik_solve',
            name='ik_solve',
            output='screen',
            parameters=[
                robot_description_semantic,
                robot_description_kinematics_path,
                
                {
                    'robot_description_topic': robot_description_topic,
                    'target_pose_topic': target_pose_topic,
                    'current_pose_topic': current_pose_topic,
                },
            ]
        ),
    ])