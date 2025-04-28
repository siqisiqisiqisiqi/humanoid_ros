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

    # # publish the passive joint value
    passive_joint_pub = launch_ros.actions.Node(
        package="servo_keyboard_py",
        executable="static_joint_publisher",
        name="static_joint_publisher",
        output="screen",
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("my_moveit_servo")
        + "/config/humanoid_servo.rviz"
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("t170a_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    right_arm_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",  # must match ros2_controllers.yaml
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
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
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[
                    {"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )

    # joint_state_pub = launch_ros.actions.Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     parameters=[
    #         moveit_config.robot_description,
    #         {"zeros": {
    #             "right_shoulder_pitch": 1.168,
    #             "right_shoulder_roll": 0,
    #             "right_elbow_roll": -0.1,
    #             "right_elbow_pitch": 0.757,
    #             "right_wrist_yaw": 0.0,
    #             "right_wrist_pitch": 0.25,
    #             "right_wrist_roll": 0.820,
    #         }},
    #     ],
    # )

    # joint_state_pub_gui = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher',    # keeps RViz panels happy
    #     output='screen',
    #     parameters=[
    #         moveit_config.robot_description,
    #         {
    #             'use_gui': True,
    #         },
    #         {"zeros": {
    #             "right_shoulder_pitch": 1.168,
    #             "right_shoulder_roll": 0,
    #             "right_elbow_roll": -0.1,
    #             "right_elbow_pitch": 0.757,
    #             "right_wrist_yaw": 0.0,
    #             "right_wrist_pitch": 0.25,
    #             "right_wrist_roll": 0.820,
    #         }},
    #     ],
    # )

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
            passive_joint_pub,
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            right_arm_controller_spawner,
            # joint_state_pub_gui,
            # joint_state_pub,
            servo_node,
            container,
        ]
    )
