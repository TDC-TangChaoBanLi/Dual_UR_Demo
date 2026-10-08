#!/usr/bin/env python3
"""Integration regression; run against an isolated, disposable MuJoCo/MoveIt domain."""
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene
from mujoco_god_plugin.srv import GetObjectPose, GetJointStates
from mujoco_dock_sim_plugin.srv import Dock
from mujoco_ros2_control_msgs.msg import InitialWeld
from mujoco_ros2_control_msgs.srv import ResetScene
from space_sim_task_control.action import DockStorage, ExitStorage
from space_sim_task_control.msg import LensInitialState
from space_sim_task_control.srv import ResetPlanningEnvironment
from std_srvs.srv import Trigger


def matrix(pose: Pose):
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return out


def main():
    rclpy.init()
    node = rclpy.create_node("reset_environment_regression")
    def wait(future, timeout=80):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        assert future.done(), "request timed out"
        return future.result()
    def call(kind, name, request):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=30), name
        try:
            return wait(client.call_async(request))
        finally:
            node.destroy_client(client)
    def reset(request=None):
        result = call(ResetPlanningEnvironment, "/reset_planning_environment",
                      request or ResetPlanningEnvironment.Request())
        assert result.success, result.message
        return result
    def pose(name):
        request = GetObjectPose.Request(type="site", name=name)
        result = call(GetObjectPose, "/mujoco_god_plugin/get_object_pose", request)
        assert result.success, result.message
        return matrix(result.pose.pose)
    def scene():
        request = GetPlanningScene.Request()
        request.components.components = 4 | 16 | 128
        return call(GetPlanningScene, "/get_planning_scene", request).scene
    def joints():
        result = call(GetJointStates, "/mujoco_god_plugin/get_joint_states", GetJointStates.Request())
        assert result.success
        return dict(zip(result.state.name, result.state.position))
    def mate(a, b):
        relative = np.linalg.inv(pose(a)) @ pose(b)
        assert np.linalg.norm(relative[:3, 3]) < 0.006, (a, b, relative[:3, 3])
        angle = math.acos(np.clip((np.trace(relative[:3, :3] @ np.diag([-1, 1, -1]))-1)/2, -1, 1))
        assert angle < 0.06, (a, b, angle)

    try:
        baseline = reset()
        home = dict(zip(baseline.joint_states.name, baseline.joint_states.position))
        assert len(baseline.lenses) == 6 and all(s.location == 0 for s in baseline.lenses)
        client = ActionClient(node, DockStorage, "/dock_storage_lens")
        assert client.wait_for_server(timeout_sec=15)
        handle = wait(client.send_goal_async(DockStorage.Goal(arm="arm_A", lens=6, grasp_face="S2")))
        assert handle.accepted
        rejected = call(ResetPlanningEnvironment, "/reset_planning_environment", ResetPlanningEnvironment.Request())
        assert not rejected.success and "仍在运行" in rejected.message
        finished = wait(handle.get_result_async(), 150).result
        assert finished.success, finished.message
        exit_client = ActionClient(node, ExitStorage, "/exit_storage")
        assert exit_client.wait_for_server(timeout_sec=15)
        exit_handle = wait(exit_client.send_goal_async(ExitStorage.Goal(arm="arm_A")))
        assert exit_handle.accepted
        finished = wait(exit_handle.get_result_async(), 180).result
        assert finished.success, finished.message
        # Use a physically demonstrated holding configuration, rather than folding
        # a large held lens into the solar panel or arm at home.
        holding = joints()
        reset()
        print("PASS: busy reset rejected; picked lens reset back to storage", flush=True)
        partial = ResetPlanningEnvironment.Request()
        partial.joint_states.name = ["arm_A_ur_shoulder_pan_joint"]
        partial.joint_states.position = [-1.3]
        custom = reset(partial)
        assert abs(dict(zip(custom.joint_states.name, custom.joint_states.position))[
            "arm_A_ur_shoulder_pan_joint"] + 1.3) < 0.02
        mixed = ResetPlanningEnvironment.Request()
        mixed.joint_states.name = list(holding)
        mixed.joint_states.position = list(holding.values())
        mixed.lenses = [LensInitialState(lens=1, location=1, centre_face="S5"),
                        LensInitialState(lens=6, location=2, arm="arm_A", lens_face="S2")]
        for _ in range(2):
            state = reset(mixed)
            assert all(abs(value-holding[name]) < 0.02 for name, value in
                       zip(state.joint_states.name, state.joint_states.position))
            mate("len_ring_1_S0", "len_center_S5")
            mate("arm_A__tcp_site", "len_ring_6_S2")
            current = scene()
            attached = current.robot_state.attached_collision_objects
            assert len(attached) == 1 and attached[0].object.id == "space_sim_carried_lens_6"
            assert attached[0].link_name == "arm_A__tcp"
            assert len(current.world.collision_objects) == 5
        print("PASS: custom joints, centre dock, arm attachment, repeated mixed reset", flush=True)

        for invalid in [
            [LensInitialState(lens=1), LensInitialState(lens=1)],
            [LensInitialState(lens=1, location=1, centre_face="S0"),
             LensInitialState(lens=2, location=1, centre_face="S0")],
            [LensInitialState(lens=1, location=2, arm="arm_A", lens_face="S2"),
             LensInitialState(lens=2, location=2, arm="arm_A", lens_face="S4")],
        ]:
            result = call(ResetPlanningEnvironment, "/reset_planning_environment",
                          ResetPlanningEnvironment.Request(lenses=invalid))
            assert not result.success
            assert len(scene().robot_state.attached_collision_objects) == 1
            mate("len_ring_1_S0", "len_center_S5")
        invalid = ResetPlanningEnvironment.Request()
        invalid.joint_states.name = ["arm_A_ur_shoulder_pan_joint"]
        invalid.joint_states.position = [float("nan")]
        assert not call(ResetPlanningEnvironment, "/reset_planning_environment", invalid).success
        invalid_scene = ResetScene.Request()
        invalid_scene.state_overrides.joint_states.name = ["arm_A_ur_shoulder_pan_joint"]
        invalid_scene.state_overrides.joint_states.position = [0.0]
        invalid_scene.welds = [InitialWeld(name="no_such_weld", active=False)]
        before = joints()
        assert not call(ResetScene, "/mujoco_ros2_control_node/reset_scene", invalid_scene).success
        assert abs(joints()["arm_A_ur_shoulder_pan_joint"] - before["arm_A_ur_shoulder_pan_joint"]) < 0.002
        mate("arm_A__tcp_site", "len_ring_6_S2")
        print("PASS: conflicting lens/centre/arm assignments and nonfinite joints rejected", flush=True)

        all_installed = ResetPlanningEnvironment.Request(lenses=[
            LensInitialState(lens=i+1, location=1, centre_face=f"S{i}") for i in range(6)])
        reset(all_installed)
        for i in range(6):
            mate(f"len_ring_{i+1}_S0", f"len_center_S{i}")
        assert not scene().robot_state.attached_collision_objects
        print("PASS: six centre interfaces initialized and verified", flush=True)

        result = call(Trigger, "/initialize_planning_environment", Trigger.Request())
        assert result.success, result.message
        current = scene()
        assert not current.robot_state.attached_collision_objects and len(current.world.collision_objects) == 6
        actual = joints()
        assert all(abs(actual[name]-value) < 0.02 for name, value in home.items())
        for lens in range(1, 7):
            dock = call(Dock, "/mujoco_dock_sim_plugin/dock", Dock.Request(
                site1=f"len_ring_{lens}_S0", site2=f"len_dock_{lens}_site", group="default"))
            assert dock.success and dock.state == "already_docked", dock.message
        print("PASS: initialize restores home, six storage welds and clears old attachment", flush=True)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
