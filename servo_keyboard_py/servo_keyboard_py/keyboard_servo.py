#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from control_msgs.msg import JointJog
import sys
import tty
import termios

# Twist keys
KEY_BINDINGS_TWIST = {
    'w': ('linear', 'x', 0.05),
    's': ('linear', 'x', -0.05),
    'a': ('linear', 'y', 0.05),
    'd': ('linear', 'y', -0.05),
    'q': ('linear', 'z', 0.05),
    'e': ('linear', 'z', -0.05),
    'i': ('angular', 'x', 0.5),
    'k': ('angular', 'x', -0.5),
    'j': ('angular', 'y', 0.5),
    'l': ('angular', 'y', -0.5),
    'u': ('angular', 'z', 0.5),
    'o': ('angular', 'z', -0.5),
}

# Joint keys
KEY_BINDINGS_JOINT = {
    '1': 'panda_joint1',
    '2': 'panda_joint2',
    '3': 'panda_joint3',
    '4': 'panda_joint4',
    '5': 'panda_joint5',
    '6': 'panda_joint6',
    '7': 'panda_joint7',
}


def get_key():
    """Get a single keypress from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class ServoKeyboardNode(Node):
    def __init__(self):
        super().__init__('humanoid_servo_keyboard_input')

        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)

        # Switch mode service
        self.cli = self.create_client(
            ServoCommandType, '/servo_node/switch_command_type')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for switch_command_type service...')

        self.command_type = 0  # Start in JOINT mode
        self.invert_joint_direction = False
        self.change_command_type(self.command_type)

        self.timer = self.create_timer(0.1, self.listen_keyboard)
        self.get_logger().info(
            "Press 'j' for joint mode, 't' for twist mode, 'r' to invert joint direction.")

    def change_command_type(self, command_type: int):
        self.get_logger().info(f"This is a test!!!")
        req = ServoCommandType.Request()
        req.command_type = command_type
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.command_type = command_type
            mode_name = 'TWIST' if command_type == 1 else 'JOINT'
            self.get_logger().info(f"Switched to {mode_name} mode.")
        else:
            self.get_logger().error("Failed to switch mode.")

    def listen_keyboard(self):
        try:
            key = get_key()
        except Exception:
            return

        # Mode switching
        self.get_logger().info(f"key value is {key}!!!")
        if key == 't':
            self.change_command_type(1)
            self.get_logger().info(f"Switched to twist mode.")
        elif key == 'j':
            self.change_command_type(0)
            self.get_logger().info(f"Switched to joint mode.")
        elif key == 'r':
            self.invert_joint_direction = not self.invert_joint_direction
            self.get_logger().info(
                f"Inverted joint direction: {self.invert_joint_direction}")
            return

        # Twist mode
        if self.command_type == 1 and key in KEY_BINDINGS_TWIST:
            cmd_type, axis, value = KEY_BINDINGS_TWIST[key]
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "panda_link0"
            setattr(getattr(twist.twist, cmd_type), axis, value)
            self.twist_pub.publish(twist)
            self.get_logger().info(f'Twist: {cmd_type} {axis} = {value}')

        # Joint mode
        elif self.command_type == 0 and key in KEY_BINDINGS_JOINT:
            joint_name = KEY_BINDINGS_JOINT[key]
            direction = -1.0 if self.invert_joint_direction else 1.0
            delta = 1 * direction

            jog = JointJog()
            jog.header.stamp = self.get_clock().now().to_msg()
            jog.joint_names = [joint_name]
            jog.velocities = [delta]
            jog.duration = 1.0

            self.joint_pub.publish(jog)
            self.get_logger().info(f'Joint: {joint_name} += {delta}')

        elif key == '\x03':  # Ctrl+C
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = ServoKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
