#!/usr/bin/env python3
"""Isolated physical fault test: release a held mirror during storage exit.

Explicitly initializes a fresh test. Never restores or teleports after injection.
Requires a task server with run_flow:=false and the hybrid execution stack.
"""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from mujoco_dock_sim_plugin.srv import Release
from space_sim_task_control.action import DockStorage, ExitStorage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initialize', action='store_true', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '11', '91'):
        raise RuntimeError('An isolated ROS domain is required')
    rclpy.init()
    node = rclpy.create_node('payload_loss_test')
    observed = {'moving': False, 'maximum_speed': float('inf'), 'stamp': 0.}
    summary = {}

    def receive(msg):
        observed['moving'] = any(abs(v) > .003 for name, v in zip(msg.name, msg.velocity)
                                 if name.startswith('arm_A_'))
        observed['maximum_speed'] = max((abs(v) for name, v in zip(msg.name, msg.velocity)
                                        if name.startswith(('arm_A_', 'arm_B_'))), default=float('inf'))
        observed['stamp'] = time.monotonic()

    subscription = node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)

    def wait(future, timeout=240):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        if not future.done():
            raise RuntimeError('Fault test operation timed out')
        return future.result()

    try:
        initialize = node.create_client(Trigger, '/initialize_planning_environment')
        assert initialize.wait_for_service(timeout_sec=20)
        assert wait(initialize.call_async(Trigger.Request()), 90).success
        pick = ActionClient(node, DockStorage, '/dock_storage_lens')
        exit_client = ActionClient(node, ExitStorage, '/exit_storage')
        assert pick.wait_for_server(timeout_sec=20) and exit_client.wait_for_server(timeout_sec=20)
        goal = DockStorage.Goal(arm='arm_A', lens=6, grasp_face='S2', phase_deg=0.)
        handle = wait(pick.send_goal_async(goal), 30)
        assert handle.accepted
        result = wait(handle.get_result_async()).result
        assert result.success, result.message
        observed['moving'] = False
        handle = wait(exit_client.send_goal_async(ExitStorage.Goal(arm='arm_A')), 30)
        assert handle.accepted
        pending = handle.get_result_async()
        end = time.monotonic() + 120
        while not observed['moving'] and not pending.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        assert observed['moving'] and not pending.done(), 'Exit motion never started'
        release = node.create_client(Release, '/mujoco_dock_sim_plugin/release')
        assert release.wait_for_service(timeout_sec=5)
        injected_at = time.monotonic()
        released = wait(release.call_async(Release.Request(
            site1='arm_A__tcp_site', site2='len_ring_6_S2')), 20)
        assert released.success, released.message
        result = wait(pending, 20).result
        summary = {'release_state': released.state, 'action_success': result.success,
                   'message': result.message, 'detection_wait_s': time.monotonic() - injected_at}
        assert not result.success and '持片检查失败' in result.message, summary
        end = time.monotonic() + 10
        while (observed['maximum_speed'] >= .001 or
               time.monotonic() - observed['stamp'] > .5) and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        summary['maximum_stopped_speed'] = observed['maximum_speed']
        assert observed['maximum_speed'] < .001 and time.monotonic() - observed['stamp'] < .5, summary
        print('PASS physical payload-loss detection:', json.dumps(summary, ensure_ascii=False), flush=True)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
