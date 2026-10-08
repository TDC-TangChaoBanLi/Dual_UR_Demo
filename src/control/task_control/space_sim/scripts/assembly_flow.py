#!/usr/bin/env python3
"""YAML driven client for the independent space_sim task actions."""

import argparse
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from space_sim_task_control.action import DockCentre, DockStorage, ExitStorage, MoveLens


class AssemblyFlow(Node):
    def __init__(self):
        super().__init__("space_sim_assembly_flow")
        self.init_client = self.create_client(Trigger, "initialize_planning_environment")
        self.action_clients = {
            "dock_storage": ActionClient(self, DockStorage, "dock_storage_lens"),
            "exit_storage": ActionClient(self, ExitStorage, "exit_storage"),
            "move_lens": ActionClient(self, MoveLens, "move_lens"),
            "dock_centre": ActionClient(self, DockCentre, "dock_centre"),
        }
        self.completed = set()
        self.install_order = []
        self.failed = threading.Event()
        self.condition = threading.Condition()
        self.active_actions = set()
        self.action_completed = {"arm_A": 0, "arm_B": 0}

    @staticmethod
    def wait_future(future, timeout, what):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError(f"{what} timed out after {timeout:.0f} s")
        return future.result()

    def initialize(self):
        if not self.init_client.wait_for_service(timeout_sec=35):
            raise RuntimeError("initialize_planning_environment unavailable")
        for attempt in range(2):
            try:
                reply = self.wait_future(
                    self.init_client.call_async(Trigger.Request()), 90, "initialize"
                )
                break
            except RuntimeError:
                if attempt == 1:
                    raise
                self.get_logger().warn("初始化响应超时，重试初始化复位服务")
        if not reply.success:
            raise RuntimeError(reply.message)
        self.get_logger().info(reply.message)

    def action(self, key, goal, timeout):
        client = self.action_clients[key]
        if not client.wait_for_server(timeout_sec=20):
            raise RuntimeError(f"{key} action server unavailable")
        with self.condition:
            self.active_actions.add(goal.arm)
            self.condition.notify_all()
        try:
            handle = self.wait_future(client.send_goal_async(goal), 30, f"{key} acceptance")
            if not handle.accepted:
                raise RuntimeError(f"{key} goal rejected for {goal.arm}")
            response = self.wait_future(handle.get_result_async(), timeout, key)
            result = response.result
            if not result.success:
                raise RuntimeError(f"{goal.arm} {key}: {result.message}")
            self.get_logger().info(f"{goal.arm} {key}: {result.message}")
            return result
        finally:
            with self.condition:
                self.active_actions.discard(goal.arm)
                self.action_completed[goal.arm] += 1
                self.condition.notify_all()

    def run_arm(self, arm, jobs, default_timeout):
        try:
            for job in jobs:
                if self.failed.is_set():
                    return
                self.wait_for(job.get("before_pick", ""))
                if self.failed.is_set():
                    return
                pick = DockStorage.Goal()
                pick.arm = arm
                pick.lens = int(job.get("lens", 0))
                pick.allowed_lenses = list(map(int, job.get("lens_candidates", [])))
                pick.grasp_face = job.get("grasp_face", "")
                pick.allowed_faces = list(job.get("grasp_faces", []))
                pick.phase_deg = float(job.get("phase_deg", 0.0))
                pick.allowed_phases_deg = list(map(float, job.get("phases_deg", [])))
                pick.randomize = bool(job.get("randomize_pick", False))
                picked = self.action("dock_storage", pick, default_timeout)

                exit_goal = ExitStorage.Goal()
                exit_goal.arm = arm
                exit_goal.hover_z = float(job.get("hover_z", 0.0))
                exit_goal.exit_distance = float(job.get("exit_distance", 0.0))
                self.action("exit_storage", exit_goal, default_timeout)
                with self.condition:
                    self.completed.add("exit:" + job.get("name", job.get("centre_face", "")))
                    self.condition.notify_all()

                for pose in job.get("waypoints", []):
                    target = PoseStamped()
                    target.header.frame_id = "world"
                    target.pose.position.x, target.pose.position.y, target.pose.position.z = pose["position"]
                    (target.pose.orientation.x, target.pose.orientation.y,
                     target.pose.orientation.z, target.pose.orientation.w) = pose["orientation_xyzw"]
                    move = MoveLens.Goal()
                    move.arm = arm
                    move.target = target
                    self.action("move_lens", move, default_timeout)

                self.wait_for(job.get("after", ""))
                if self.failed.is_set():
                    return
                place = DockCentre.Goal()
                place.arm = arm
                place.centre_face = job.get("centre_face", "")
                place.allowed_faces = list(job.get("centre_faces", []))
                place.randomize = bool(job.get("randomize_centre", False))
                place.return_home = bool(job.get("return_home", False))
                retries = max(0, int(job.get("dock_retries", 2)))
                for attempt in range(retries + 1):
                    try:
                        finished = self.action("dock_centre", place, default_timeout)
                        break
                    except RuntimeError as exc:
                        message = str(exc)
                        # dock/release 已发生后不得重放装配动作；只对规划失败
                        # 以当前真实关节状态重新求解，镜片始终由本臂持有。
                        planning_failure = any(text in message for text in (
                            "IK:", "CHOMP", "STOMP", "笛卡尔路径", "双臂轨迹在",
                            "规划失败", "确定性直连"
                        ))
                        if attempt == retries or not planning_failure or any(text in message for text in (
                            "镜片已装配", "中心已对接", "接口对合误差超限", "末端已释放"
                        )):
                            raise
                        self.get_logger().warn(
                            f"[流程回退] arm={arm} 镜片=len_ring_{picked.lens} "
                            f"阶段=中心装配 目标={place.centre_face or place.allowed_faces} "
                            f"原因={message}；方案=按实时状态重新调用DockCentre，"
                            f"重试={attempt + 1}/{retries}"
                        )
                        peer = "arm_B" if arm == "arm_A" else "arm_A"
                        with self.condition:
                            if peer in self.active_actions:
                                completed = self.action_completed[peer]
                                self.condition.wait_for(
                                    lambda: self.action_completed[peer] != completed or
                                    self.failed.is_set(), timeout=20.0
                                )
                            else:
                                self.condition.wait(timeout=1.0)
                if finished.lens != picked.lens:
                    raise RuntimeError(f"{arm}: picked lens {picked.lens}, docked {finished.lens}")
                with self.condition:
                    self.completed.add(finished.centre_face)
                    self.install_order.append(finished.centre_face)
                    self.condition.notify_all()
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.failed.set()
            with self.condition:
                self.condition.notify_all()

    def wait_for(self, prerequisite):
        with self.condition:
            while prerequisite and prerequisite not in self.completed and not self.failed.is_set():
                self.condition.wait(timeout=1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flow", required=True, type=Path)
    args, _ros_args = parser.parse_known_args()
    config = yaml.safe_load(args.flow.read_text())
    if set(config["arms"]) != {"arm_A", "arm_B"}:
        parser.error("flow must define arm_A and arm_B")
    rclpy.init()
    node = AssemblyFlow()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spinning = threading.Thread(target=executor.spin, daemon=True)
    spinning.start()
    try:
        node.initialize()
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(node.run_arm, arm, jobs, float(config.get("action_timeout_s", 300)))
                       for arm, jobs in config["arms"].items()]
            for future in futures:
                future.result()
        expected = set(config.get("expected_faces", [f"S{i}" for i in range(6)]))
        completed_faces = {face for face in node.completed if face.startswith("S")}
        if (node.failed.is_set() or completed_faces != expected or
                len(node.install_order) != len(expected) or
                len(set(node.install_order)) != len(node.install_order)):
            raise RuntimeError(
                f"flow incomplete: completed={sorted(completed_faces)}, "
                f"install_order={node.install_order}, expected={sorted(expected)}"
            )
        expected_prefix = list(config.get("expected_prefix", []))
        if node.install_order[:len(expected_prefix)] != expected_prefix:
            raise RuntimeError(
                f"installation order mismatch: actual={node.install_order}, "
                f"expected_prefix={expected_prefix}"
            )
        node.get_logger().info(
            f"flow completed: {sorted(completed_faces)}; install_order={node.install_order}"
        )
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 2
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spinning.join(timeout=2)


if __name__ == "__main__":
    sys.exit(main())
