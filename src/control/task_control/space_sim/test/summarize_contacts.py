#!/usr/bin/env python3
"""Summarize sampled MuJoCo contacts without treating docking as collision-free."""
import argparse
import json
from pathlib import Path
import re


def read_stages(path):
    metadata = {'start': None, 'insert': {}, 'grasp': {}, 'faces': {},
                'storage_cleared': {}, 'tool_cleared': {}}
    if not path.exists():
        return metadata
    for line in path.open():
        event = json.loads(line)
        message = event['message']
        if '[动作开始]' in message and metadata['start'] is None:
            metadata['start'] = event['t']
        lens = re.search(r'镜片=len_ring_(\d+)', message)
        arm = re.search(r'arm=(arm_[AB])', message)
        if not lens or not arm:
            continue
        if '[动作完成]' in message and 'action=dock_storage_lens ' in message:
            metadata['storage_cleared'].setdefault(lens[1], event['t'])
        if (('[阶段]' in message and '(return_home)' in message) or
                ('[动作完成]' in message and 'action=dock_centre ' in message)):
            metadata['tool_cleared'].setdefault((arm[1], lens[1]), event['t'])
        if '[阶段]' not in message:
            continue
        if '(grasp_axis)' in message:
            metadata['grasp'].setdefault((arm[1], lens[1]), event['t'])
        if '(centre_insert)' in message:
            metadata['insert'].setdefault(lens[1], event['t'])
            face = re.search(r'len_center_S(\d)', message)
            if face:
                metadata['faces'][lens[1]] = int(face[1])
    return metadata


def summarize(path, stages=None):
    pairs = {}
    snapshots = 0
    ignored = 0
    for line in path.open():
        snapshot = json.loads(line)
        snapshots += 1
        if stages and stages['start'] is not None and snapshot['t'] < stages['start']:
            ignored += 1
            continue
        for contact in snapshot['contacts']:
            key = tuple(sorted(contact['bodies']))
            row = pairs.setdefault(key, {
                'bodies': list(key), 'contact_samples': 0,
                'minimum_distance_m': 0.0, 'maximum_normal_force_n': 0.0,
                'first_stamp': snapshot['t'], 'last_stamp': snapshot['t'],
            })
            row['contact_samples'] += 1
            row['minimum_distance_m'] = min(row['minimum_distance_m'], contact['distance'])
            row['maximum_normal_force_n'] = max(row['maximum_normal_force_n'], abs(contact['wrench'][0]))
            row['last_stamp'] = snapshot['t']
    for key, row in pairs.items():
        storage = re.fullmatch(r'len_dock_(\d+)', key[0])
        if storage and key[1] == 'len_ring_' + storage[1]:
            row['category'] = 'storage_interface'
        elif any(body.endswith('_gripper_dock') for body in key) and any(
                body.startswith('len_ring_') for body in key):
            row['category'] = 'tool_interface_requires_stage_review'
        elif all(body.startswith(('len_ring_', 'len_center')) for body in key):
            row['category'] = 'mirror_interface_requires_stage_review'
        else:
            row['category'] = 'unexpected_pair_requires_review'
        if stages:
            rings = [body.removeprefix('len_ring_') for body in key if body.startswith('len_ring_')]
            threshold = None
            until = None
            adjacency = True
            if row['category'] == 'storage_interface':
                until = stages['storage_cleared'].get(storage[1])
                row['stage_consistent'] = (until is not None and
                    row['last_stamp'] <= until+.1 and row['minimum_distance_m'] >= -.001)
            elif row['category'] == 'tool_interface_requires_stage_review':
                arm = next(body.removesuffix('_gripper_dock') for body in key if body.endswith('_gripper_dock'))
                threshold = stages['grasp'].get((arm, rings[0]))
                until = stages['tool_cleared'].get((arm, rings[0]))
            elif row['category'] == 'mirror_interface_requires_stage_review':
                times = [stages['insert'].get(lens) for lens in rings]
                if times and all(value is not None for value in times):
                    threshold = max(times)
                if len(rings) == 2:
                    a, b = (stages['faces'].get(lens) for lens in rings)
                    adjacency = a is not None and b is not None and (a-b) % 6 in (1, 5)
            if row['category'] != 'storage_interface':
                row['stage_consistent'] = (threshold is not None and adjacency and
                    row['first_stamp'] >= threshold-.1 and row['minimum_distance_m'] >= -.001)
                if row['category'] == 'tool_interface_requires_stage_review':
                    row['stage_consistent'] &= until is not None and row['last_stamp'] <= until+.1
                row['earliest_expected_stamp'] = threshold
            row['latest_expected_stamp'] = until
    return {'snapshots': snapshots, 'pre_action_snapshots_ignored': ignored, 'pairs': list(pairs.values()),
            'scope': 'Sampled observations. Interface categories require event/geometry review; '
                     'they are not automatic allowances or a continuous collision-free proof.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record_directory', type=Path)
    args = parser.parse_args()
    result = summarize(args.record_directory / 'contacts.jsonl',
                       read_stages(args.record_directory / 'events.jsonl'))
    (args.record_directory / 'contact_summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
