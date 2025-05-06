#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StaticJointPublisher(Node):
    def __init__(self):
        super().__init__('static_joint_publisher')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
            "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
            "right_knee_pitch", "right_ankle_pitch", "right_ankle_roll",
            "waist_roll", "waist_yaw", "waist_pitch",
            "head_yaw", "head_pitch", "head_roll",
            "left_index_1_joint", "left_little_1_joint", "left_middle_1_joint",
            "left_ring_1_joint", "left_thumb_1_joint", "left_thumb_2_joint",
            "right_index_1_joint", "right_little_1_joint", "right_middle_1_joint",
            "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"
        ]
        msg.position = [0.0] * len(msg.name)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
