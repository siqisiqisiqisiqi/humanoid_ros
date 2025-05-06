#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class HandJointPublisher(Node):
    def __init__(self):
        super().__init__('hand_joint_publisher')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.subscription = self.create_subscription(
            Bool,
            '/hand_state',  # This matches MoveIt Servo output
            self.listener_callback,
            10
        )
        self.pub = self.create_publisher(JointState, 'joint_command', 10)

    def listener_callback(self, msg: Bool):
        hand_state = msg.data
        send_msg = JointState()
        send_msg.header.stamp = self.get_clock().now().to_msg()
        send_msg.name = [
            "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
            "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
        ]
        if hand_state:
            send_msg.position = [1.19, 1.06, 1.12, 1.13, 0.81, 0.25]
        else:
            send_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(send_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
