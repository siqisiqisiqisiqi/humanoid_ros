# Humanoid MoveIt Servo Workspace

This repository contains a ROS 2 workspace for real-time control of a humanoid robot's arm using MoveIt Servo, keyboard teleoperation, and custom MoveIt and URDF configuration.

## Repository Structure

- `my_moveit_servo/` - Customized MoveIt Servo package for humanoid control.
- `servo_keyboard_py/` - Python node to control the robot arm with keyboard input.
- `t170a_arm_moveit_config/` - MoveIt configuration package for the T170A humanoid robot arm.
- `T170A/` - URDF (Unified Robot Description Format) and robot description files.

## Features

- Real-time joint-space control with MoveIt Servo
- Keyboard-based Cartesian jog commands
- Custom MoveIt motion planning and kinematic configuration
- Full URDF model for visualization and planning
- Ready to integrate advanced vision or language-based control later

## Requirements

- ROS 2 Humble (or your ROS 2 distro)
- MoveIt 2
- MoveIt Servo
- Python 3

## How to Build

```bash
# Clone the repository
git clone https://github.com/your_username/humanoid_moveit_servo_ws.git
cd humanoid_moveit_servo_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build
source install/setup.bash
