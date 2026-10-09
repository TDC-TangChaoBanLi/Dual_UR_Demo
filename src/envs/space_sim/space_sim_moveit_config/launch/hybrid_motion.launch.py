"""Deploy generic multi_arm_motion on the space_sim robot.

Start MuJoCo controllers and move_group separately. Do not run assembly's direct
controller execution concurrently with this controller owner.
"""
from pathlib import Path
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Path(get_package_share_directory('space_sim_description'))
    config = Path(get_package_share_directory('space_sim_moveit_config')) / 'config'
    urdf = subprocess.check_output(['xacro', str(description/'xacro/space_sim.xacro'),
        'include_movable_lenses:=false', 'force_abs_paths:=true', 'safety_limits:=false'], text=True)
    parameters = {
        'robot_description': urdf,
        'robot_description_semantic': (config/'space_sim.srdf').read_text(),
        'robot_description_kinematics': yaml.safe_load((config/'kinematics.yaml').read_text()),
        'robot_description_planning': yaml.safe_load((config/'joint_limits.yaml').read_text()),
        'group_name': 'arm_BOTH', 'use_sim_time': True,
        'planning_scene_service': '/get_planning_scene',
        'mpc.enable_execution': ParameterValue(LaunchConfiguration('enable_execution'), value_type=bool),
        'mpc.dt': ParameterValue(LaunchConfiguration('control_period'), value_type=float),
        'mpc.horizon': 5,
        'mpc.optimization_budget': ParameterValue(LaunchConfiguration('optimization_budget'), value_type=float),
        'mpc.clearance': .005, 'mpc.influence_distance': .02,
        'mpc.tracking_weight': 10000.,
        'mpc.max_state_age': .3,
        'execution.command_lead': ParameterValue(LaunchConfiguration('command_lead'), value_type=float),
        'execution.goal_timeout': 90., 'execution.tracking_limit': .2,
        'execution.reference_lag_tolerance': ParameterValue(LaunchConfiguration('reference_lag_tolerance'), value_type=float),
        'execution.joint_tolerance': ParameterValue(LaunchConfiguration('joint_tolerance'), value_type=float),
        'execution.seed_duration_scale': 2.0,
        'execution.reference_duration_scale': 3.0,
        'execution.stopped_velocity': .0005,
        'execution.continuous_desired_commands': True,
        'dynamic_stomp.num_timesteps': 40, 'dynamic_stomp.delta_t': .1,
        'dynamic_stomp.minimum_clearance': ParameterValue(LaunchConfiguration('planning_clearance'), value_type=float),
        'dynamic_stomp.random_seed': 7,
        'dynamic_stomp.num_iterations': 100, 'dynamic_stomp.num_iterations_after_valid': 0,
    }
    router = {'input_topic': '/mpc_joint_commands', 'controllers': ['arm_A', 'arm_B']}
    suffixes = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
    for arm in ['arm_A', 'arm_B']:
        router[f'{arm}.joints'] = [f'{arm}_ur_{suffix}_joint' for suffix in suffixes]
        router[f'{arm}.topic'] = f'/{arm}_ur_joint_trajectory_controller/joint_trajectory'
    return LaunchDescription([
        DeclareLaunchArgument('enable_execution', default_value='false'),
        DeclareLaunchArgument('joint_tolerance', default_value='0.0002'),
        DeclareLaunchArgument('control_period', default_value='0.2'),
        DeclareLaunchArgument('command_lead', default_value=PythonExpression([LaunchConfiguration('control_period'), '*0.8'])),
        DeclareLaunchArgument('optimization_budget', default_value='0.05'),
        DeclareLaunchArgument('planning_clearance', default_value='0.008',
                             description='Global path clearance in metres; local execution retains 0.005 m'),
        DeclareLaunchArgument('reference_lag_tolerance', default_value='0.0',
                             description='Optional independent reference progress adjustment; 0 disables'),
        Node(package='multi_arm_hybrid_planning', executable='coordinator', output='screen', parameters=[parameters]),
        Node(package='multi_arm_hybrid_planning', executable='command_router', output='screen', parameters=[router, {'use_sim_time': True}]),
    ])
