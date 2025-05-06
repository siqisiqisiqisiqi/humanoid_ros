#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class TestROS2Bridge(Node):
    def __init__(self):
        super().__init__("motor_control")
        self.get_logger().info("ROS2 Bridge Node Initialized.")

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )

        self.target_joint = ["left_shoulder_pitch", "left_shoulder_roll", 
                                "left_elbow_yaw", "left_elbow_pitch",
                                "left_wrist_yaw", "left_wrist_pitch", "left_wrist_roll"]

        self.publisher_ = None
        self.joint_name = None
        self.init_position = None
        self.started = False  # Flag to trigger setup once

    def setup_motion(self):
        self.get_logger().info("Setting up motion controller...")

        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        target_indices = [i for i, item in enumerate(self.joint_name) if item in self.target_joint]

        self.joint_state = JointState()
        self.joint_state.name = [self.joint_name[i] for i in target_indices]

        num_joints = len(self.joint_state.name)

        self.joint_state.position = np.array([0.0] * num_joints, dtype=np.float64).tolist()
        self.default_joints = [self.init_position[i] for i in target_indices]

        self.max_joints = np.array(self.default_joints) + 0.2
        self.min_joints = np.array(self.default_joints) - 0.2

        self.time_start = time.time()
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.started = True
        self.get_logger().info("Motion control setup complete.")

    def listener_callback(self, msg):
        if not self.started:
            self.get_logger().info(f'Received joint state: {msg.name}')
            self.get_logger().info(f'Received joint position: {msg.position}')
            if self.joint_name is None:
                self.joint_name = msg.name
            if self.init_position is None:
                self.init_position = msg.position

            if self.joint_name and self.init_position:
                self.setup_motion()

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_position = (
            np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
        )
        self.joint_state.position = joint_position.tolist()

        self.publisher_.publish(self.joint_state)
        # self.get_logger().info(f'Publishing: {joint_position}')


def main(args=None):
    rclpy.init(args=args)
    node = TestROS2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
