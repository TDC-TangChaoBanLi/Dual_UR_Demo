#!/usr/bin/env python3
"""Replay a logged free-space reference against the current scene, without reset.

The current joints must match the logged start; the coordinator validates that
and all future motion. Contact references require their original task constraints
and are deliberately rejected here.
"""
import argparse
import json
from pathlib import Path
import re
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from multi_arm_motion_interfaces.action import ExecuteMotion
from moveit_msgs.msg import Constraints, JointConstraint
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-log', required=True, type=Path)
    parser.add_argument('--context', required=True, help='Substring identifying the logged movement')
    parser.add_argument('--arm', required=True, choices=['arm_A', 'arm_B'])
    parser.add_argument('--occurrence', type=int, default=0)
    parser.add_argument('--planning-time', type=float, default=30.)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    references = []
    lenses = set()
    for line in args.task_log.open():
        if '[平滑轨迹记录]' in line and args.context in line and f'arm={args.arm} ' in line:
            if any(token in line for token in ('插入2cm', '撤退', '接近2cm', '下降', '镜面竖直')):
                raise ValueError('Only unconstrained free-space references can be replayed')
            references.append(json.loads(line.split(' data=', 1)[1]))
            lens = re.search(r'镜片=(len_ring_\d+)', line)
            if lens:
                lenses.add(lens[1])
    if len(lenses) > 1:
        raise ValueError('context matches different mirrors; include the exact mirror ID')
    record = references[args.occurrence]
    if not 0 < args.planning_time <= 30:
        raise ValueError('planning-time must be in (0,30]')
    rclpy.init()
    node = rclpy.create_node('replay_hybrid_reference', parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        client = ActionClient(node, ExecuteMotion, '/execute_motion')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('coordinator unavailable')
        goal = ExecuteMotion.Goal()
        goal.optimize_reference = True
        goal.timeout = 600.
        goal.reference.joint_names = record['joints']
        for value in record['points']:
            point = JointTrajectoryPoint()
            point.positions = value['q']
            ns = round(value['t']*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1000000000)
            goal.reference.points.append(point)
        goal.request.group_name = args.arm
        goal.request.allowed_planning_time = args.planning_time
        goal.request.max_velocity_scaling_factor = .1
        goal.request.max_acceleration_scaling_factor = .1
        goal.request.start_state.is_diff = True
        constraint = Constraints()
        for name, q in zip(record['joints'], record['points'][-1]['q']):
            constraint.joint_constraints.append(JointConstraint(
                joint_name=name, position=q, tolerance_above=.0002, tolerance_below=.0002, weight=1.))
        goal.request.goal_constraints = [constraint]

        def wait(future, timeout):
            deadline = time.monotonic()+timeout
            while not future.done() and time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if not future.done():
                raise TimeoutError('action response timed out; inspect coordinator before further commands')
            return future.result()

        handle = wait(client.send_goal_async(goal), 15)
        if handle is None or not handle.accepted:
            raise RuntimeError('goal rejected')
        try:
            response = wait(handle.get_result_async(), 610)
        except TimeoutError:
            wait(handle.cancel_goal_async(), 10)
            raise
        result = {'status': response.status, 'error_code': response.result.error_code.val,
                  'message': response.result.message, 'planning_time': args.planning_time,
                  'source_log': str(args.task_log), 'context': args.context, 'occurrence': args.occurrence}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)
        if response.status != 4 or response.result.error_code.val != 1:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
