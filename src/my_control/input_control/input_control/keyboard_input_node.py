#!/usr/bin/env python3
import math
import re
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw (rad) to quaternion (x, y, z, w)
    Rotation order: XYZ intrinsic == RPY
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('keyboard_input_node')

        self.pub_target_pose = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.sub_current_pose = self.create_subscription(
            PoseStamped, '/current_pose', self.current_pose_callback, 10
        )

        # Parameters
        self.declare_parameter('frame_id', 'arm_A__tcp')
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('pos_step', 0.001)      # m per key press
        self.declare_parameter('rot_step_deg', 2.0)    # deg per key press

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.pos_step = float(self.get_parameter('pos_step').value)
        self.rot_step = math.radians(float(self.get_parameter('rot_step_deg').value))

        # Target pose state
        self.target_x = 0.6
        self.target_y = 0.8
        self.target_z = 1.88

        self.target_roll = 1.5708  # 90 degrees
        self.target_pitch = 0.0
        self.target_yaw = 3.14159  # 180 degrees

        # Current pose state
        self.current_x = 0.6
        self.current_y = 0.8
        self.current_z = 1.88

        self.current_roll = 1.5708  # 90 degrees
        self.current_pitch = 0.0
        self.current_yaw = 3.14159  # 180 degrees

        self.running = True
        self.start_pub = False

        # Terminal settings for non-blocking key read
        self.stdin_fd = sys.stdin.fileno()
        self.old_termios = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.print_help()
        self.get_logger().info('Keyboard input node started (terminal stdin mode).')

    def print_help(self):
        help_text = """
================ Keyboard Control ================
Position:
  w / s : x+ / x-
  a / d : y+ / y-
  r / f : z+ / z-

Orientation:
  u / j : roll+ / roll-
  i / k : pitch+ / pitch-
  o / l : yaw+ / yaw-

Other:
  space : print current target/current pose
  x     : set target pose to current pose
  q     : quit
==================================================
"""
        self.get_logger().info(help_text)

    def read_key_nonblocking(self):
        """
        Non-blocking read of one key from current terminal.
        Returns:
            None if no input
            single-character string otherwise
        """
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not dr:
            return None

        ch = sys.stdin.read(1)

        # Handle common escape sequences for arrow keys if needed
        # Up    : \x1b[A
        # Down  : \x1b[B
        # Right : \x1b[C
        # Left  : \x1b[D
        if ch == '\x1b':
            dr, _, _ = select.select([sys.stdin], [], [], 0.0)
            if dr:
                ch2 = sys.stdin.read(1)
                dr, _, _ = select.select([sys.stdin], [], [], 0.0)
                if dr:
                    ch3 = sys.stdin.read(1)
                    return ch + ch2 + ch3
            return ch

        return ch

    def handle_key(self, ch):
        if ch is None:
            return

        # Space
        if ch == ' ':
            self.print_current_pose()
            return

        # Normalize to lowercase for letter keys
        if len(ch) == 1:
            ch = ch.lower()

        # Special commands
        if ch == 'x':
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.target_roll = self.current_roll
            self.target_pitch = self.current_pitch
            self.target_yaw = self.current_yaw
            self.get_logger().info('Target pose set to current pose')
            return

        if ch == 'q':
            self.get_logger().info('Quit requested by keyboard.')
            self.running = False
            return

        # Position increments
        if ch == 'w':
            self.target_x += self.pos_step
        elif ch == 's':
            self.target_x -= self.pos_step
        elif ch == 'a':
            self.target_y += self.pos_step
        elif ch == 'd':
            self.target_y -= self.pos_step
        elif ch == 'r':
            self.target_z += self.pos_step
        elif ch == 'f':
            self.target_z -= self.pos_step

        # Orientation increments
        elif ch == 'u':
            self.target_roll += self.rot_step
        elif ch == 'j':
            self.target_roll -= self.rot_step
        elif ch == 'i':
            self.target_pitch += self.rot_step
        elif ch == 'k':
            self.target_pitch -= self.rot_step
        elif ch == 'o':
            self.target_yaw += self.rot_step
        elif ch == 'l':
            self.target_yaw -= self.rot_step

        # Wrap angles to [-pi, pi]
        self.target_roll = math.atan2(math.sin(self.target_roll), math.cos(self.target_roll))
        self.target_pitch = math.atan2(math.sin(self.target_pitch), math.cos(self.target_pitch))
        self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))

    def build_pose_msg(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z

        qx, qy, qz, qw = euler_to_quaternion(
            self.target_roll,
            self.target_pitch,
            self.target_yaw
        )
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion (x, y, z, w) to roll, pitch, yaw (rad)
        Rotation order: XYZ intrinsic == RPY
        """
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def current_pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id == 'arm_A__tcp':
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
            self.current_z = msg.pose.position.z

            self.current_roll, self.current_pitch, self.current_yaw = self.quaternion_to_euler(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
            if not self.start_pub:
                self.get_logger().info('First pose received, setting target pose to current pose')
                self.target_x = self.current_x
                self.target_y = self.current_y
                self.target_z = self.current_z
                self.target_roll = self.current_roll
                self.target_pitch = self.current_pitch
                self.target_yaw = self.current_yaw
                self.start_pub = True

    def print_current_pose(self):
        self.get_logger().info(
            f"Target/Current:\n"
            f"  position = [{self.target_x:.4f}/{self.current_x:.4f}, "
            f"{self.target_y:.4f}/{self.current_y:.4f}, "
            f"{self.target_z:.4f}/{self.current_z:.4f}] m\n"
            f"  rpy      = [{math.degrees(self.target_roll):.2f}/{math.degrees(self.current_roll):.2f}, "
            f"{math.degrees(self.target_pitch):.2f}/{math.degrees(self.current_pitch):.2f}, "
            f"{math.degrees(self.target_yaw):.2f}/{math.degrees(self.current_yaw):.2f}] deg"
        )

    def timer_callback(self):
        if not self.running:
            self.get_logger().info('Shutting down keyboard_input_node...')
            rclpy.shutdown()
            return
        self.print_current_pose()
        if not self.start_pub:
            return
        ch = self.read_key_nonblocking()
        self.handle_key(ch)

        msg = self.build_pose_msg()
        self.pub_target_pose.publish(msg)

    def destroy_node(self):
        try:
            termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_termios)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()