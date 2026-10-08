#!/usr/bin/env python3
"""Record task diagnostics with scene snapshots for offline planning reproduction."""
import argparse
import json
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Log
from moveit_msgs.srv import GetPlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("space_sim_planning_audit")
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    events = (output / "events.jsonl").open("w")
    sequence = 0

    def receive(msg):
        nonlocal sequence
        if not any(token in msg.msg for token in (
            "[运动开始]", "[动作开始]", "[阶段]", "回退]", "失败]", "[动作完成]",
        )):
            return
        sequence += 1
        record = {"id": sequence, "stamp": message_to_ordereddict(msg.stamp),
                  "logger": msg.name, "level": msg.level, "message": msg.msg}
        events.write(json.dumps(record, ensure_ascii=False) + "\n")
        events.flush()
        if not client.service_is_ready():
            return
        request = GetPlanningScene.Request()
        request.components.components = 2 | 4 | 8 | 16 | 128 | 256 | 512
        future = client.call_async(request)

        def save(done):
            try:
                result = done.result()
                snapshot = dict(record)
                snapshot["scene"] = message_to_ordereddict(result.scene)
                (output / f"scene_{record['id']:04d}.json").write_text(
                    json.dumps(snapshot, ensure_ascii=False))
            except Exception as exc:
                node.get_logger().error(f"Snapshot {record['id']} failed: {exc}")
        future.add_done_callback(save)

    subscription = node.create_subscription(Log, "/rosout", receive, 200)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(subscription)
        events.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
