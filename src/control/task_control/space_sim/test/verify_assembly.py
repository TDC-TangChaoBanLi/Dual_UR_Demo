#!/usr/bin/env python3
"""Read-only assembly verification and sampled execution recording in an isolated domain."""
import argparse
import itertools
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetStateValidity, GetPlanningScene
from mujoco_god_plugin.srv import GetObjectPose, GetContacts
from rcl_interfaces.msg import Log


def matrix(pose):
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return result


def command_continuity(rows):
    """Compare each replacement start with the previous checked cubic prefix."""
    previous, errors = {}, []
    for row in rows:
        prior = previous.get(row['arm'])
        previous[row['arm']] = row
        if prior is None:
            continue
        t = row['start']-prior['start']
        for a, b in zip(prior['prefix'], prior['prefix'][1:]):
            if a['t'] <= t <= b['t']:
                dt = b['t']-a['t']
                u = (t-a['t'])/dt
                q = ((2*u**3-3*u*u+1)*np.array(a['q']) +
                     (u**3-2*u*u+u)*dt*np.array(a['v']) +
                     (-2*u**3+3*u*u)*np.array(b['q']) +
                     (u**3-u*u)*dt*np.array(b['v']))
                errors.append(float(np.max(np.abs(q-np.array(row['prefix'][0]['q'])))))
                break
    return {'checks': len(errors), 'maximum_joint_start_jump': max(errors, default=0.),
            'p95_joint_start_jump': float(np.percentile(errors, 95)) if errors else 0.}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--record', action='store_true', help='Record until Ctrl-C; otherwise verify final assembly')
    parser.add_argument('--until-flow-end', action='store_true',
                        help='Stop recording automatically on flow completed/incomplete')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '11', '91'):
        raise RuntimeError('Use an isolated ROS domain')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('assembly_verification', parameter_overrides=[Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    subscriptions = []
    summary = {}
    try:
        if args.record:
            state = [None]
            pending = [None]
            contact_pending = [None]
            counts = {'samples': 0, 'both_moving': 0, 'collision_checks': 0, 'invalid_states': [],
                      'physics_contact_snapshots': 0}
            stamps = {'arm_A': set(), 'arm_B': set()}
            poses = {}
            flow_ended = [False]
            validity = node.create_client(GetStateValidity, '/check_state_validity')
            contact_client = node.create_client(GetContacts, '/mujoco_god_plugin/get_contacts')
            with (args.output/'motion.jsonl').open('w') as motion, (args.output/'events.jsonl').open('w') as events, (args.output/'commands.jsonl').open('w') as commands, (args.output/'contacts.jsonl').open('w') as contacts:
                def receive(msg):
                    state[0] = msg

                def event(msg):
                    if any(token in msg.msg for token in ('[运动', '[动作', '[阶段]', '[回退]', 'latched stopped', 'committed;', 'flow completed', 'flow incomplete')):
                        events.write(json.dumps({'t': node.get_clock().now().nanoseconds*1e-9,
                            'source_stamp': msg.stamp.sec+msg.stamp.nanosec*1e-9,
                            'logger': msg.name, 'level': msg.level, 'message': msg.msg}, ensure_ascii=False)+'\n')
                        events.flush()
                    if args.until_flow_end and any(token in msg.msg for token in ('flow completed:', 'flow incomplete:')):
                        flow_ended[0] = True

                def sample():
                    msg = state[0]
                    if msg is None:
                        return
                    counts['samples'] += 1
                    velocities = dict(zip(msg.name, msg.velocity))
                    counts['both_moving'] += int(all(any(abs(v) > .001 for n, v in velocities.items() if n.startswith(arm+'_')) for arm in stamps))
                    motion.write(json.dumps({'t': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        'names': list(msg.name), 'q': list(msg.position), 'v': list(msg.velocity), 'poses': poses})+'\n')
                    if counts['samples'] % 2 == 0 and contact_client.service_is_ready() and contact_pending[0] is None:
                        contact_pending[0] = contact_client.call_async(GetContacts.Request())
                        def received_contacts(future):
                            response=future.result()
                            if response.success:
                                counts['physics_contact_snapshots'] += 1
                                contacts.write(json.dumps({'t': response.header.stamp.sec+response.header.stamp.nanosec*1e-9,
                                    'contacts': [{'geoms': list(c.geom_names), 'bodies': list(c.body_names),
                                        'distance': c.distance, 'wrench': list(c.wrench_contact_frame)} for c in response.contacts]})+'\n')
                            contact_pending[0] = None
                        contact_pending[0].add_done_callback(received_contacts)
                    if counts['samples'] % 20 == 0 and validity.service_is_ready() and pending[0] is None:
                        request = GetStateValidity.Request()
                        request.group_name = 'arm_BOTH'
                        request.robot_state.is_diff = True
                        request.robot_state.joint_state = msg
                        pending[0] = validity.call_async(request)

                        def checked(future):
                            response = future.result()
                            counts['collision_checks'] += 1
                            if not response.valid:
                                counts['invalid_states'].append({'t': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                    'contacts': [[c.contact_body_1, c.contact_body_2] for c in response.contacts]})
                            pending[0] = None
                        pending[0].add_done_callback(checked)

                subscriptions.append(node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data))
                subscriptions.append(node.create_subscription(Log, '/rosout', event, 200))
                for arm in stamps:
                    def receive_command(msg, arm=arm):
                        stamps[arm].add((msg.header.stamp.sec, msg.header.stamp.nanosec))
                        commands.write(json.dumps({'arm': arm, 'received': node.get_clock().now().nanoseconds*1e-9,
                            'start': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                            'joints': list(msg.joint_names), 'prefix': [
                                {'t': p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                                 'q': list(p.positions), 'v': list(p.velocities)} for p in msg.points[:3]]})+'\n')
                    subscriptions.append(node.create_subscription(JointTrajectory,
                        f'/{arm}_ur_joint_trajectory_controller/joint_trajectory',
                        receive_command, 100))
                for name in ['arm_A__tcp_site', 'arm_B__tcp_site']+[f'len_ring_{i}' for i in range(1, 7)]:
                    kind = 'body' if name.startswith('len_ring') else 'site'
                    def receive_pose(msg, name=name):
                        p=msg.pose
                        poses[name]={'stamp': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                            'xyz': [p.position.x,p.position.y,p.position.z],
                            'xyzw': [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]}
                    subscriptions.append(node.create_subscription(PoseStamped,
                        f'/mujoco_god_plugin/pose/{kind}/{name}', receive_pose, qos_profile_sensor_data))
                timer = node.create_timer(.05, sample)
                try:
                    while rclpy.ok() and not flow_ended[0]:
                        executor.spin_once(timeout_sec=.2)
                except KeyboardInterrupt:
                    pass
                finally:
                    summary.update(counts)
                    summary['matching_command_stamps'] = len(stamps['arm_A'] & stamps['arm_B'])
                    commands.flush()
                    with (args.output/'commands.jsonl').open() as recorded:
                        summary['command_continuity'] = command_continuity(json.loads(line) for line in recorded)
                    (args.output/'motion_summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
        else:
            def call(kind, service, request):
                client = node.create_client(kind, service)
                if not client.wait_for_service(timeout_sec=15):
                    raise RuntimeError(f'{service} unavailable')
                future = client.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=15)
                if not future.done():
                    raise TimeoutError(service)
                response = future.result()
                node.destroy_client(client)
                return response

            def pose(site):
                response = call(GetObjectPose, '/mujoco_god_plugin/get_object_pose', GetObjectPose.Request(type='site', name=site))
                if not response.success:
                    raise RuntimeError(response.message)
                return matrix(response.pose.pose)

            lenses = [pose(f'len_ring_{i}_S0') for i in range(1, 7)]
            centres = [pose(f'len_center_S{i}') for i in range(6)]
            assignment = min(itertools.permutations(range(6)), key=lambda faces:
                sum(np.linalg.norm(lenses[i][:3, 3]-centres[f][:3, 3]) for i, f in enumerate(faces)))
            pairs = []
            for i, face in enumerate(assignment):
                relative = lenses[i][:3, :3].T @ centres[face][:3, :3]
                angle = math.acos(float(np.clip((np.trace(np.diag([-1., 1., -1.]).T @ relative)-1)/2, -1, 1)))
                pairs.append({'lens': i+1, 'centre_face': f'S{face}',
                    'position_error_m': float(np.linalg.norm(lenses[i][:3, 3]-centres[face][:3, 3])),
                    'orientation_error_rad': angle})
            request = GetPlanningScene.Request()
            request.components.components = 4 | 16 | 128
            scene = call(GetPlanningScene, '/get_planning_scene', request).scene
            objects = {o.id for o in scene.world.collision_objects}
            attached = [o.object.id for o in scene.robot_state.attached_collision_objects]
            validity_request=GetStateValidity.Request(group_name='arm_BOTH')
            validity_request.robot_state.is_diff=True
            valid = call(GetStateValidity, '/check_state_validity', validity_request)
            summary = {'pairs': pairs, 'attached_objects': attached, 'world_objects': sorted(objects), 'final_state_valid': valid.valid}
            summary['success'] = (not attached and valid.valid and
                all(f'space_sim_carried_lens_{i}' in objects for i in range(1, 7)) and
                all(p['position_error_m'] < .001 and p['orientation_error_rad'] < .01 for p in pairs))
            (args.output/'final_assembly.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
            print(json.dumps(summary, ensure_ascii=False), flush=True)
            if not summary['success']:
                return 2
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
