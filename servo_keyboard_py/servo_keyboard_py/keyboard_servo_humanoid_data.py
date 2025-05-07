#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from pynput import keyboard as pynput_keyboard
from control_msgs.msg import JointJog
import numpy as np
import time

# Key mappings for twist commands
KEY_BINDINGS_TWIST = {
    'w': ('linear', 'x', 0.2),
    's': ('linear', 'x', -0.2),
    'a': ('linear', 'y', 0.2),
    'd': ('linear', 'y', -0.2),
    'q': ('linear', 'z', 0.2),
    'e': ('linear', 'z', -0.2),
    'i': ('angular', 'x', 0.2),
    'k': ('angular', 'x', -0.2),
    'j': ('angular', 'y', 0.2),
    'l': ('angular', 'y', -0.2),
    'u': ('angular', 'z', 0.2),
    'o': ('angular', 'z', -0.2),
}


class ServoTwistTeleop(Node):
    def __init__(self):
        super().__init__('servo_twist_pynput')
        # self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
        #                                                rclpy.Parameter.Type.BOOL,
        #                                                True)])
        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)
        self.hand_pub = self.create_publisher(Bool, '/hand_state', 10)

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Service client for switching command type
        self.cli = self.create_client(
            ServoCommandType, '/servo_node/switch_command_type')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for switch_command_type service...')
        self.req = ServoCommandType.Request()
        self.change_command_type(new_type=1)

        # State
        self.hand_state = False
        self.pressed_keys = set()

        # Start pynput listener in a background thread
        self.listener = pynput_keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        # Timer for publishing twist at fixed rate
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.get_logger().info(
            "Pynput-based keyboard teleop started. Use WASDQE + IJKLUO, 'h' to toggle hand. Press ESC to exit.")

        # Home parameter
        self.joint_name = ["right_shoulder_pitch", "right_shoulder_roll",
                           "right_elbow_yaw", "right_elbow_pitch",
                           "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll"
                           ]
        self.joint_home_value = np.array(
            [0.11, 0.04, 0.10, 1.63, 0.06, 0.00, 0.00])
        self.joint_values = {name: 0.0 for name in self.joint_name}

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_name:
                self.joint_values[name] = position
        self.joint_states = np.array(
            [self.joint_values[name] for name in self.joint_name])

    def change_command_type(self, new_type: int):
        self.req.command_type = new_type
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.handle_switch_response)
        self.command_type = new_type

    def handle_switch_response(self, future):
        try:
            response = future.result()
            if response.success:
                mode_names = {0: 'JOINT', 1: 'TWIST'}
                mode = mode_names.get(self.command_type, 'UNKNOWN')
                self.get_logger().info(f"Switched to {mode} mode.")
            else:
                self.get_logger().error(
                    f"Failed to switch command type: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def go_home(self):
        # change to the joint motion
        self.change_command_type(new_type=0)
        # control the joint to the home position
        jog = JointJog()
        jog.joint_names = self.joint_name
        while np.linalg.norm(self.joint_home_value - self.joint_states) > 0.1:
            delta = self.joint_home_value - self.joint_states
            jog.header.stamp = self.get_clock().now().to_msg()
            jog.velocities = (1.0 * delta).tolist()
            jog.duration = 0.1
            self.joint_pub.publish(jog)
            time.sleep(0.05)
        self.change_command_type(new_type=1)

    def on_press(self, key):
        # Handle character keys
        try:
            char = key.char.lower()
        except AttributeError:
            # Handle special keys
            if key == pynput_keyboard.Key.esc:
                self.get_logger().info('ESC pressed, shutting down.')
                rclpy.shutdown()
            return

        if char in KEY_BINDINGS_TWIST:
            self.pressed_keys.add(char)
            self.get_logger().info(f'Detected the keyboard input {char}.')

        elif char == 'g':
            # Toggle hand state
            msg = Bool()
            msg.data = not self.hand_state
            self.hand_pub.publish(msg)
            self.hand_state = not self.hand_state
            self.get_logger().info(f"Hand state toggled to {self.hand_state}.")

        elif char == 'h':
            # Toggle hand state
            self.go_home()
            self.get_logger().info(f"Move the robot back to home.")

    def on_release(self, key):
        try:
            char = key.char.lower()
        except AttributeError:
            return

        if char in self.pressed_keys:
            self.pressed_keys.discard(char)

    def timer_callback(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        value = 0
        # Reset twist
        # Only set values for pressed keys
        for char in list(self.pressed_keys):
            cmd_type, axis, value = KEY_BINDINGS_TWIST[char]
            setattr(getattr(twist.twist, cmd_type), axis, value)

        # Publish twist (zero if no key pressed)
        self.twist_pub.publish(twist)
        if value != 0:
            self.get_logger().info(f'published value is {value}.')

    def destroy_node(self):
        # Stop listener cleanly
        if self.listener.running:
            self.listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoTwistTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
