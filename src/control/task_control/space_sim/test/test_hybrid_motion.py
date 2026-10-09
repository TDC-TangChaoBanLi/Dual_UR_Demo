#!/usr/bin/env python3
"""Exercise generic ExecuteMotion against the space_sim MuJoCo controllers.

Requires isolated simulation, move_group and hybrid_motion.launch.py. Records
actual joint motion overlap and both routed controller command start stamps.
"""
import json
import os
from pathlib import Path
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetStateValidity
from multi_arm_motion_interfaces.action import ExecuteMotion


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--offset', type=float, default=.04)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '11', '91'):
        raise RuntimeError('Use an isolated ROS domain')
    rclpy.init()
    node = Node('hybrid_motion_acceptance')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    joint = {}
    records, stamps = [], {'arm_A': [], 'arm_B': []}

    def state(msg):
        joint.update(zip(msg.name, msg.position))
        values = dict(zip(msg.name, msg.velocity))
        records.append({'t': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        'positions': dict(zip(msg.name, msg.position)), 'velocities': values})

    subscription = node.create_subscription(JointState, '/joint_states', state, qos_profile_sensor_data)
    commands = []
    for arm in stamps:
        commands.append(node.create_subscription(JointTrajectory, f'/{arm}_ur_joint_trajectory_controller/joint_trajectory',
            lambda msg, arm=arm: stamps[arm].append([msg.header.stamp.sec, msg.header.stamp.nanosec]), 10))

    def wait(future, timeout=90):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        if not future.done():
            raise TimeoutError('Action did not complete')
        return future.result()

    result_summary = {}
    try:
        client = ActionClient(node, ExecuteMotion, '/execute_motion')
        end = time.monotonic()+20
        while (not client.server_is_ready() or len(joint)<12) and time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
        assert client.server_is_ready() and len(joint)>=12, f'server ready={client.server_is_ready()}, joint count={len(joint)}'
        futures, targets = [], {}
        for arm, sign in [('arm_A', 1.), ('arm_B', -1.)]:
            goal = ExecuteMotion.Goal()
            goal.request.group_name = arm
            goal.request.allowed_planning_time = 2.
            goal.request.max_velocity_scaling_factor = .2
            goal.request.max_acceleration_scaling_factor = .2
            goal.timeout = 80.
            target = {name: value for name, value in joint.items() if name.startswith(arm+'_ur_')}
            target[f'{arm}_ur_wrist_3_joint'] += sign*args.offset
            targets.update(target)
            goal.request.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
                joint_name=name, position=value, tolerance_above=.0001, tolerance_below=.0001, weight=1.)
                for name, value in target.items()])]
            futures.append(client.send_goal_async(goal))
        handles = [wait(f) for f in futures]
        assert all(h.accepted for h in handles), 'goal rejected'
        result_futures = [h.get_result_async() for h in handles]
        results = [wait(f) for f in result_futures]
        result_summary['results'] = [{'code': result.result.error_code.val, 'message': result.result.message} for result in results]
        assert all(result.result.error_code.val == 1 for result in results), result_summary
        error = max(abs(joint[name]-value) for name, value in targets.items())
        overlap = sum(any(abs(v)>.001 for name, v in r['velocities'].items() if name.startswith('arm_A_')) and
                      any(abs(v)>.001 for name, v in r['velocities'].items() if name.startswith('arm_B_')) for r in records)
        paired = len(set(map(tuple, stamps['arm_A'])) & set(map(tuple, stamps['arm_B'])))
        result_summary.update(error=error, overlap_samples=overlap, matched_command_stamps=paired)
        assert error<.011 and overlap>10 and paired>10, result_summary
        validity = node.create_client(GetStateValidity, '/check_state_validity')
        assert validity.wait_for_service(timeout_sec=5), 'MoveIt collision validation service unavailable'
        # Check measured intermediate states, in addition to executor's future
        # sweeps. Freeze the captured list while live feedback continues.
        captured = records[:]
        checked = 0
        for record in captured[::max(1, len(captured)//40)]:
            request = GetStateValidity.Request()
            request.group_name = 'arm_BOTH'
            request.robot_state.is_diff = True
            request.robot_state.joint_state.name = list(record['positions'])
            request.robot_state.joint_state.position = list(record['positions'].values())
            response = wait(validity.call_async(request), timeout=5)
            assert response.valid, [(c.contact_body_1, c.contact_body_2) for c in response.contacts]
            checked += 1
        result_summary['measured_collision_checks'] = checked
        print('PASS articulated dual-arm MuJoCo:', json.dumps(result_summary), flush=True)
    finally:
        Path(args.output).write_text(json.dumps({'summary': result_summary, 'joint_states': records, 'command_stamps': stamps}))
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
