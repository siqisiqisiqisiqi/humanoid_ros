#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading


class JointStateRepublisher(Node):
    def __init__(self):
        super().__init__('joint_state_republisher')
        # self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
        #                                                rclpy.Parameter.Type.BOOL,
        #                                                True)])
        self.latest_msg = None
        self.lock = threading.Lock()

        # Subscriber to Isaac Sim's joint_states
        self.create_subscription(
            JointState, '/joint_states_isaac', self.joint_state_callback, 10)

        # Publisher to republish at 100 Hz
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Timer for 100 Hz
        self.create_timer(0.01, self.timer_callback)

    def joint_state_callback(self, msg):
        with self.lock:
            self.latest_msg = msg

    def timer_callback(self):
        with self.lock:
            if self.latest_msg:
                self.latest_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(self.latest_msg)


def main():
    rclpy.init()
    node = JointStateRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
