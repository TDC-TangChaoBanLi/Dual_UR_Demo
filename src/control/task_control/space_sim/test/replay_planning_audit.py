#!/usr/bin/env python3
"""Replay failed planning candidates in a disposable, paused MoveIt domain.

Uses recorded scenes and full joint/target DEBUG records. Does not execute motion.
STOMP replay uses its default seed; original Cartesian seed is not recorded.
"""
import argparse
import copy
import json
import math
import re
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK, GetStateValidity, GetCartesianPath, GetPlanningScene
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict


def parse(directory):
    snapshots = [json.loads(p.read_text()) for p in (directory / "scenes").glob("scene_*.json")]
    by_message = {s["message"]: s for s in sorted(snapshots, key=lambda s: s["id"])}
    current = {arm: {} for arm in ("arm_A", "arm_B")}
    pending = {arm: [] for arm in current}
    cases = []
    tokens = ("[IK候选失败]", "[规划回退]", "[终点候选失败]", "[规划初值回退]",
              "[碰撞协调回退]", "[回退]", "[笛卡尔路径不完整]")
    for line_number, line in enumerate((directory / "task_run.log").read_text().splitlines(), 1):
        arm_match = re.search(r"arm=(arm_[AB])", line)
        if not arm_match:
            continue
        arm = arm_match[1]
        state = current[arm]
        if "[运动开始]" in line:
            current[arm] = state = {"arm": arm}
            pending[arm] = []
        if "[运动状态]" in line:
            joints = re.search(r"完整机器人关节\(rad\)=\{([^}]+)\}", line)
            if joints:
                state["start"] = {name: float(value) for name, value in
                                  (item.split(":") for item in joints[1].split(","))}
        if "[请求目标]" in line:
            joints = re.search(r"关节\(rad\)=\{([^}]+)\}", line)
            if joints:
                state["goal"] = {name: float(value) for name, value in
                                 (item.split(":") for item in joints[1].split(","))}
            pose = re.search(r"xyz\(m\)=\[([^]]+)\] quat\(xyzw\)=\[([^]]+)\]", line)
            if pose:
                state["pose"] = [float(value) for value in (pose[1] + "," + pose[2]).split(",")]
        if "[IK种子]" in line:
            joints = re.search(r"关节\(rad\)=\{([^}]+)\}", line)
            if joints:
                state["seed"] = {name: float(value) for name, value in
                                 (item.split(":") for item in joints[1].split(","))}
        if "[规划成功]" in line and "goal" in state:
            for failed in pending[arm]:
                failed.setdefault("following_success_goal", copy.deepcopy(state["goal"]))
        if not any(token in line for token in tokens):
            continue
        message = line.split("]: ", 1)[-1]
        scene = by_message.get(message)
        snapshot_exact = scene is not None
        if scene is None:
            timestamp = re.search(r"\[(\d+\.\d+)\]", line)
            if not timestamp:
                continue
            before = [s for s in snapshots if s["stamp"]["sec"] + s["stamp"]["nanosec"]*1e-9 <= float(timestamp[1])]
            if not before:
                continue
            scene = max(before, key=lambda s: (s["stamp"]["sec"], s["stamp"]["nanosec"]))
        case = copy.deepcopy(state)
        case.update(line=line_number, message=message, scene_id=scene["id"], scene=scene["scene"],
                    snapshot_exact_event=snapshot_exact)
        cases.append(case)
        pending[arm].append(case)
    return cases


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--case", type=int, action="append", help="case index, 0 based")
    parser.add_argument("--smooth-repeats", type=int, default=0,
                        help="Replay coordination candidates with the diagnostic TOTG/Ruckig service")
    args = parser.parse_args()
    cases = parse(args.directory)
    (args.directory / "cases.json").write_text(json.dumps(cases, ensure_ascii=False, indent=2))
    rclpy.init()
    node = rclpy.create_node("space_sim_failure_replay")
    clients = {}

    def call(kind, name, request, timeout=35):
        client = clients.setdefault(name, node.create_client(kind, name)) if name not in clients else clients[name]
        assert client.wait_for_service(timeout_sec=30), name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        assert future.done(), name + " timeout"
        return future.result()

    def validity(state):
        state = copy.deepcopy(state)
        state.is_diff = True
        state.attached_collision_objects = []  # Scene already owns the recorded attachments.
        req = GetStateValidity.Request(robot_state=state, group_name="arm_BOTH")
        result = call(GetStateValidity, "/check_state_validity", req)
        return {"valid": result.valid, "contacts": sorted(set(
            c.contact_body_1 + " <-> " + c.contact_body_2 for c in result.contacts))}

    def check_segments(jt, base, positions):
        for segment in range(1, len(jt.points)):
            a, b = jt.points[segment-1].positions, jt.points[segment].positions
            checks = max(1, math.ceil(max(abs(x-y) for x, y in zip(a, b))/.015))
            for step in range(checks+1):
                sampled = copy.deepcopy(base)
                values = dict(positions)
                values.update({n: x+step/checks*(y-x) for n, x, y in zip(jt.joint_names, a, b)})
                sampled.joint_state.name = list(values)
                sampled.joint_state.position = list(values.values())
                valid = validity(sampled)
                if not valid["valid"]:
                    return {"segment": segment, "fraction": step/checks, **valid, "joints": values}
        return None

    previous_results = args.directory / "replay_results.json"
    results = json.loads(previous_results.read_text()) if args.case and previous_results.exists() else []
    try:
        for index, case in enumerate(cases):
            if args.case is not None and index not in args.case:
                continue
            old_request = GetPlanningScene.Request()
            old_request.components.components = 4 | 8
            old_scene = call(GetPlanningScene, "/get_planning_scene", old_request).scene
            clear = PlanningScene(is_diff=True)
            clear.robot_state.is_diff = True
            for old in old_scene.robot_state.attached_collision_objects:
                old.object.operation = CollisionObject.REMOVE
                clear.robot_state.attached_collision_objects.append(old)
            if clear.robot_state.attached_collision_objects:
                assert call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=clear)).success
            # Detaching may put geometry back into the world; clear world objects
            # after detachment, before loading the next recorded scene.
            old_scene = call(GetPlanningScene, "/get_planning_scene", old_request).scene
            clear = PlanningScene(is_diff=True)
            clear.robot_state.is_diff = True
            for old in old_scene.world.collision_objects:
                old.operation = CollisionObject.REMOVE
                clear.world.collision_objects.append(old)
            if clear.world.collision_objects:
                assert call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=clear)).success
            scene = PlanningScene()
            fields = copy.deepcopy(case["scene"])
            # rosidl's JSON converter represents ROS octet scalars as one-char strings.
            for obj in fields["world"]["collision_objects"]:
                obj["operation"] = obj["operation"].encode("latin1")
            for attached in fields["robot_state"]["attached_collision_objects"]:
                obj = attached["object"]
                obj["operation"] = obj["operation"].encode("latin1")
            set_message_fields(scene, fields)
            scene.is_diff = True  # All prior geometry was explicitly cleared.
            scene.robot_state.is_diff = True
            assert call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=scene)).success
            base = copy.deepcopy(scene.robot_state)
            attachments = copy.deepcopy(base.attached_collision_objects)
            base.attached_collision_objects = []
            base.is_diff = True
            positions = dict(zip(base.joint_state.name, base.joint_state.position))
            positions.update(case.get("start", {}))
            base.joint_state.name = list(positions)
            base.joint_state.position = list(positions.values())
            base.joint_state.velocity = []
            base.joint_state.effort = []
            base.joint_state.header.stamp.sec = 0
            base.joint_state.header.stamp.nanosec = 0
            result = {"case": index, "line": case["line"], "message": case["message"],
                      "scene_id": case["scene_id"], "start": validity(base)}
            if "pose" in case:
                target = Pose()
                target.position.x, target.position.y, target.position.z = case["pose"][:3]
                target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = case["pose"][3:]
                result["ik"] = []
                for avoid in (False, True):
                    req = GetPositionIK.Request()
                    req.ik_request.group_name = case["arm"]
                    req.ik_request.ik_link_name = case["arm"] + "__tcp"
                    req.ik_request.robot_state = copy.deepcopy(base)
                    if "seed" in case:
                        seeded = dict(positions)
                        seeded.update(case["seed"])
                        req.ik_request.robot_state.joint_state.name = list(seeded)
                        req.ik_request.robot_state.joint_state.position = list(seeded.values())
                    req.ik_request.pose_stamped.header.frame_id = "world"
                    req.ik_request.pose_stamped.pose = target
                    req.ik_request.avoid_collisions = avoid
                    req.ik_request.timeout.sec = 1
                    answer = call(GetPositionIK, "/compute_ik", req)
                    item = {"avoid_collisions": avoid, "code": answer.error_code.val}
                    if answer.error_code.val == 1:
                        item["state"] = validity(answer.solution)
                    result["ik"].append(item)
                if "参考完成率" in case["message"] or "笛卡尔路径不完整" in case["message"]:
                    req = GetCartesianPath.Request()
                    req.header.frame_id = "world"
                    req.start_state = base
                    req.group_name = case["arm"]
                    req.link_name = case["arm"] + "__tcp"
                    req.waypoints = [target]
                    req.max_step = 0.001 if "笛卡尔路径不完整" in case["message"] else 0.005
                    result["cartesian"] = []
                    for avoid in (False, True):
                        req.avoid_collisions = avoid
                        answer = call(GetCartesianPath, "/compute_cartesian_path", req)
                        item = {"avoid_collisions": avoid, "fraction": answer.fraction, "code": answer.error_code.val}
                        if not avoid:
                            jt = answer.solution.joint_trajectory
                            for k, point in enumerate(jt.points):
                                sample = copy.deepcopy(base)
                                values = dict(positions)
                                values.update(zip(jt.joint_names, point.positions))
                                sample.joint_state.name = list(values)
                                sample.joint_state.position = list(values.values())
                                valid = validity(sample)
                                if not valid["valid"]:
                                    item["first_collision"] = {"point": k, **valid}
                                    break
                        result["cartesian"].append(item)
            if "goal" in case and "IK候选失败" not in case["message"]:
                endpoint = copy.deepcopy(base)
                goal_positions = dict(positions)
                goal_positions.update(case["goal"])
                endpoint.joint_state.name = list(goal_positions)
                endpoint.joint_state.position = list(goal_positions.values())
                result["endpoint"] = validity(endpoint)
                # Recheck the same joint straight line used by the fallback.
                count = max(2, math.ceil(max(abs(case["goal"][n] - positions[n]) for n in case["goal"]) / .015))
                for point in range(count + 1):
                    sampled = copy.deepcopy(base)
                    values = dict(positions)
                    values.update({n: positions[n] + point/count*(v-positions[n]) for n, v in case["goal"].items()})
                    sampled.joint_state.name = list(values)
                    sampled.joint_state.position = list(values.values())
                    valid = validity(sampled)
                    if not valid["valid"]:
                        result["direct"] = {"sample": point, "segments": count, **valid}
                        break
                else:
                    result["direct"] = {"valid": True, "segments": count}
                if ("STOMP" in case["message"] or "碰撞协调回退" in case["message"]) and "终点候选失败" not in case["message"] and result["start"]["valid"]:
                    req = GetMotionPlan.Request()
                    plan = req.motion_plan_request
                    plan.pipeline_id = "stomp"
                    plan.planner_id = "space_sim_stomp/StompPlanner"
                    plan.group_name = case["arm"]
                    plan.start_state = base
                    plan.allowed_planning_time = 12.0
                    plan.num_planning_attempts = 3
                    plan.max_velocity_scaling_factor = 0.1
                    plan.max_acceleration_scaling_factor = 0.1
                    plan.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
                        joint_name=n, position=v, tolerance_above=1e-4, tolerance_below=1e-4, weight=1.)
                        for n, v in case["goal"].items()])]
                    if "镜面竖直" in case["message"]:
                        held = next(a for a in attachments if a.link_name == case["arm"] + "__tcp")
                        q = held.object.pose.orientation
                        normal = [2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y)]
                        plan.path_constraints.name = "space_sim_mirror_vertical " + " ".join(map(str, normal)) + " 0.1"
                        oc = OrientationConstraint(link_name=case["arm"] + "__tcp", weight=1.,
                            absolute_x_axis_tolerance=math.pi+.001, absolute_y_axis_tolerance=math.pi+.001,
                            absolute_z_axis_tolerance=math.pi+.001)
                        oc.header.frame_id = "world"
                        oc.orientation.w = 1.
                        plan.path_constraints.orientation_constraints = [oc]
                    answer = call(GetMotionPlan, "/plan_kinematic_path", req)
                    result["stomp_default_seed"] = {"code": answer.motion_plan_response.error_code.val,
                        "time": answer.motion_plan_response.planning_time}
                    if "碰撞协调回退" in case["message"] and answer.motion_plan_response.error_code.val == 1:
                        jt = answer.motion_plan_response.trajectory.joint_trajectory
                        collision = None
                        for segment in range(1, len(jt.points)):
                            a, b = jt.points[segment-1].positions, jt.points[segment].positions
                            checks = max(1, math.ceil(max(abs(x-y) for x, y in zip(a, b))/.015))
                            for step in range(checks+1):
                                sampled = copy.deepcopy(base)
                                values = dict(positions)
                                values.update({n: x+step/checks*(y-x) for n, x, y in zip(jt.joint_names, a, b)})
                                sampled.joint_state.name = list(values)
                                sampled.joint_state.position = list(values.values())
                                valid = validity(sampled)
                                if not valid["valid"]:
                                    collision = {"segment": segment, "fraction": step/checks, **valid}
                                    break
                            if collision:
                                break
                        result["stomp_segment_collision"] = collision
                        if args.smooth_repeats:
                            result["smoothing_trials"] = []
                            for trial in range(args.smooth_repeats):
                                if trial:
                                    answer = call(GetMotionPlan, "/plan_kinematic_path", req)
                                raw = answer.motion_plan_response.trajectory
                                item = {"trial": trial, "plan_code": answer.motion_plan_response.error_code.val}
                                if item["plan_code"] != 1:
                                    result["smoothing_trials"].append(item)
                                    continue
                                item["raw_collision"] = check_segments(raw.joint_trajectory, base, positions)
                                smooth = GetMotionPlan.Request()
                                smooth.motion_plan_request.start_state = base
                                smooth.motion_plan_request.group_name = case["arm"]
                                smooth.motion_plan_request.trajectory_constraints.constraints = [Constraints(
                                    joint_constraints=[JointConstraint(joint_name=n, position=v)
                                        for n, v in zip(raw.joint_trajectory.joint_names, point.positions)])
                                    for point in raw.joint_trajectory.points]
                                timed = call(GetMotionPlan, "/audit_time_parameterize", smooth)
                                item["smoothing_code"] = timed.motion_plan_response.error_code.val
                                if item["smoothing_code"] == 1:
                                    item["smoothed_collision"] = check_segments(
                                        timed.motion_plan_response.trajectory.joint_trajectory, base, positions)
                                result["smoothing_trials"].append(item)
                                if item.get("smoothed_collision"):
                                    (args.directory / f"coordination_case_{index}_trajectories.json").write_text(json.dumps({
                                        "raw": message_to_ordereddict(raw),
                                        "smoothed": message_to_ordereddict(timed.motion_plan_response.trajectory),
                                        "result": item}, ensure_ascii=False))
                                    break
                    if case.get("following_success_goal") != case["goal"] and case.get("following_success_goal"):
                        plan.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
                            joint_name=n, position=v, tolerance_above=1e-4, tolerance_below=1e-4, weight=1.)
                            for n, v in case["following_success_goal"].items()])]
                        answer = call(GetMotionPlan, "/plan_kinematic_path", req)
                        result["stomp_alternative_ik"] = {"code": answer.motion_plan_response.error_code.val,
                            "time": answer.motion_plan_response.planning_time}
            results = [old for old in results if old["case"] != index] + [result]
            results.sort(key=lambda old: old["case"])
            (args.directory / "replay_results.json").write_text(json.dumps(results, ensure_ascii=False, indent=2))
            print(json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
