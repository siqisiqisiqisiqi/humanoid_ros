#!/usr/bin/env python3

import sys
import time
import numpy as np

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointSliderBridge(Node):
    def __init__(self):
        super().__init__("joint_slider_bridge")

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            JointState, 'joint_command', 10)

        self.target_joint = [
            "right_shoulder_pitch", "right_shoulder_roll",
            "right_elbow_yaw", "right_elbow_pitch",
            "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll"
        ]

        # left
        # self.limit = np.array([[-2.09, 0.087], [-0.52, 2.96], [-1.57, 1.57],
        #                       [-1.92, 0], [-3.14, 3.14], [-0.52, 0.52], [-1.57, 0.52]])

        # right
        self.limit = np.array([[-0.087, 2.09], [-2.96, 0.52], [-1.57, 1.57],
                              [0, 2.61], [-3.14, 3.14], [-0.52, 0.52], [-0.52, 1.57]])
        self.joint_limits = dict(zip(self.target_joint, self.limit))

        self.joint_name = None
        self.init_position = None
        self.target_indices = []
        self.default_joints = []
        self.gui_started = False

    def joint_state_callback(self, msg):
        if not self.gui_started:
            self.get_logger().info("Received joint state message.")

            if self.joint_name is None:
                self.joint_name = msg.name
            if self.init_position is None:
                self.init_position = msg.position

            if self.joint_name and self.init_position:
                self.target_indices = [i for i, item in enumerate(
                    self.joint_name) if item in self.target_joint]
                self.default_joints = [self.init_position[i]
                                       for i in self.target_indices]
                self.joint_state_msg = JointState()
                self.joint_state_msg.name = [
                    self.joint_name[i] for i in self.target_indices]
                self.joint_state_msg.position = [
                    0.0 for _ in self.target_indices]

                self.gui_started = True
                self.launch_gui()

    def launch_gui(self):
        app = QApplication(sys.argv)
        gui = JointSliderGUI(self)
        gui.show()
        app.exec_()

    def update_joint_position_scaled(self, index, slider_val, min_val, max_val):
        resolution = 1000
        new_position = min_val + \
            (slider_val / resolution) * (max_val - min_val)

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position[index] = new_position
        self.publisher_.publish(self.joint_state_msg)


class JointSliderGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.setWindowTitle("Joint Position Sliders")

        for i, joint in enumerate(self.ros_node.joint_state_msg.name):
            lower, upper = self.ros_node.joint_limits.get(joint, (-0.2, 0.2))
            resolution = 1000
            default_pos = self.ros_node.default_joints[i]

            # Create label that shows the value and limits
            value_label = QLabel(
                f"{joint}: {default_pos:.2f} (limits: {lower:.1f} to {upper:.1f})")

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(resolution)
            slider.setValue(int((default_pos - lower) /
                            (upper - lower) * resolution))

            def make_callback(joint_index, min_val, max_val, label_ref):
                def callback(val):
                    scaled_val = min_val + \
                        (val / resolution) * (max_val - min_val)
                    label_ref.setText(
                        f"{self.ros_node.joint_state_msg.name[joint_index]}: {scaled_val:.2f} (limits: {min_val:.1f} to {max_val:.1f})")
                    self.ros_node.update_joint_position_scaled(
                        joint_index, val, min_val, max_val)
                return callback

            slider.valueChanged.connect(
                make_callback(i, lower, upper, value_label))

            layout.addWidget(value_label)
            layout.addWidget(slider)

        self.setLayout(layout)


def main():
    rclpy.init()
    node = JointSliderBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
