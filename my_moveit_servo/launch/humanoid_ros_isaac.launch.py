import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("t170a_arm")
        .robot_description(file_path="config/T170A.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Launch Servo as a standalone node or as a "node component" for better latency/efficiency
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="false"
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("my_moveit_servo")
        .yaml("config/humanoid_servo_config.yaml")
        .to_dict()
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "right_arm"}

    initial_pose_pub = launch_ros.actions.Node(
        package="humanoid_control",
        executable="initial_pose_publisher",
        name="initial_pose_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    trajectory_to_jointstate_bridge_node = launch_ros.actions.Node(
        package="humanoid_control",
        executable="trajectory_to_jointstate_bridge",
        name="trajectory_to_jointstate_bridge",
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    joint_state_rate_adapter_node = launch_ros.actions.Node(
        package="humanoid_control",
        executable="joint_state_rate_adapter",
        name="joint_state_rate_adapter",
        output="screen",
        # parameters=[{'use_sim_time': True}],
    )

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="my_moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    # {"use_sim_time": True},
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description,
                            # {"use_sim_time": True},
                            ],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[
                    {"child_frame_id": "/base_link", "frame_id": "/world"},
                    # {"use_sim_time": True},
                ],
            ),
        ],
        output="screen",
    )

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="my_moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    return launch.LaunchDescription(
        [
            trajectory_to_jointstate_bridge_node,
            joint_state_rate_adapter_node,
            initial_pose_pub,
            servo_node,
            container,
        ]
    )
