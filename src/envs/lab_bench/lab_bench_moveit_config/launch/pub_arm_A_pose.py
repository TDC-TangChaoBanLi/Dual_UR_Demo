#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PosePub(Node):
    def __init__(self):
        super().__init__("test_arm_A_pose_pub")
        self.pub = self.create_publisher(
            PoseStamped,
            "/arm_A_servo_node/pose_target_cmds",
            10,
        )

        # 先用一个很小的位姿目标测试
        # 这里的数值要改成你当前末端附近的位置，不要一开始给很远目标
        self.x = 0.35
        self.y = 0.7
        self.z = 1.3

        self.timer = self.create_timer(0.02, self.timer_cb)  # 50 Hz

    def timer_cb(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        # 先保持一个固定姿态，这里只是示例四元数
        # 实际建议用当前末端姿态附近的四元数
        msg.pose.orientation.x = 0.5
        msg.pose.orientation.y = -0.5
        msg.pose.orientation.z = -0.5
        msg.pose.orientation.w = 0.5

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PosePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()