#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import threading
import time


class TrajectoryToJointStateBridge(Node):
    def __init__(self):
        super().__init__('trajectory_to_joint_state_bridge')

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',  # This matches MoveIt Servo output
            self.trajectory_callback,
            10
        )
        self.subscription = self.create_subscription(
            Bool,
            '/hand_state',  # This matches MoveIt Servo output
            self.hand_state_callback,
            10
        )
        self.publisher = self.create_publisher(
            JointState,
            '/joint_command',  # Isaac Sim Subscribe Node will listen to this
            10
        )

        self.create_timer(0.01, self.publish_joint_command)

        self.get_logger().info('Trajectory to JointState Bridge Node Started')

        self.joint_states = {}
        self.lock = threading.Lock()
        self.isaac_sim = False

    def hand_state_callback(self, msg: Bool):
        with self.lock:
            hand_state = msg.data
            joint_name = [
                "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
                "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
            ]
            if hand_state:
                position = [0.55, 0.80, 0.6, 0.60, 1.16, 0.13]
                velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                position = [0.0, 0.0, 0.0, 0.0, 1.2, 0.0]
                velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.get_logger().info('This is a test to show the hand state publish!!!')
            for name, pos, vel in zip(joint_name, position, velocity):
                self.joint_states[name] = [pos, vel]

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn('Received trajectory with no points!')
            return
        point = msg.points[0]
        with self.lock:
            debug_flag = False
            for name, pos, vel in zip(msg.joint_names, point.positions, point.velocities):
                self.joint_states[name] = [pos, vel]
                if vel != 0:
                    debug_flag = True
            if debug_flag:
                self.get_logger().info('This is a test to show the joint value updated!!!')

    def publish_joint_command(self):
        with self.lock:
            if not self.joint_states:
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.name = list(self.joint_states.keys())
            # self.get_logger().info(f'Joint name is {msg.name}.')
            states = list(self.joint_states.values())
            msg.position = [x[0] for x in states]
            msg.velocity = [x[1] for x in states]
            self.publisher.publish(msg)
            self.joint_states = {}


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToJointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
