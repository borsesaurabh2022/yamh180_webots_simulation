# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
import os
import pathlib

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

package_name = "mh180_simulation"


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_standalone_node = LaunchConfiguration("launch_standalone_node")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval  #

    package_dir = get_package_share_directory(launch_configuration["namespace"] + "_simulation")

    def load_config(filename):
        return pathlib.Path(os.path.join(package_dir, "config", filename)).read_text()

    def load_urdf(filename):
        return pathlib.Path(os.path.join(package_dir, "urdf", filename)).read_text()

    def load_yaml(filename):
        return yaml.safe_load(load_config(filename))

    with open(
        os.path.join(
            get_package_share_directory(package_name),
            "urdf",
            "motoman_mh180_prefix.urdf",
        )
    ) as f:
        mh12_description = f.read()
    description = {"robot_description": mh12_description}
    description_semantic = {"robot_description_semantic": load_config("motoman_mh_180_120.srdf")}
    description_kinematics = {"robot_description_kinematics": load_yaml("kinematics.yaml")}
    description_joint_limits = {"robot_description_planning": load_yaml("joint_limits.yaml")}
    sim_time = {"use_sim_time": use_sim_time}

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("mh180_simulation").yaml("config/mh180_servo_config.yaml").to_dict()
    }

    # This filter parameter should be >1. Increase it for greater smoothing but slower motion.
    low_pass_filter_coeff = {"butterworth_filter_coeff": 1.5}

    # RViz
    rviz_config_file = get_package_share_directory("mh180_simulation") + "/rviz/moveit_servo.rviz"

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[
            description,
            description_semantic,
            description_kinematics,
            description_joint_limits,
            sim_time,
        ],
        remappings=[("/monitored_planning_scene", "/mh180/servo_node/publish_planning_scene")],
        namespace=namespace,
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        condition=UnlessCondition(launch_standalone_node),
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoServer",
                name="servo_server",
                parameters=[
                    servo_params,
                    description,
                    description_kinematics,
                    description_semantic,
                    description_joint_limits,
                    low_pass_filter_coeff,
                    sim_time,
                ],
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "mh180/base_link", "frame_id": "world"}],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            description,
            description_semantic,
            description_kinematics,
            description_joint_limits,
            low_pass_filter_coeff,
            sim_time,
        ],
        output="screen",
        condition=IfCondition(launch_standalone_node),
        remappings=[
            (
                "/motoman_mh180_controller/joint_trajectory",
                "/mh180/motoman_mh180_controller/joint_trajectory",
            ),
            ("/joint_states", "/mh180/joint_states"),
        ],
        namespace=namespace,
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf2_broadcaster",
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "mh180/base_link", "world"],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="teleop_control",
        output="log",
        namespace=namespace,
        parameters=[sim_time],
    )

    nodes_to_start = [rviz_node, servo_node, static_tf, joy_node, container]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="mh180",
            description="Type/series of used UR robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )

    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz"))

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_standalone_node",
            default_value="true",
            description="launch as standalone node",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
