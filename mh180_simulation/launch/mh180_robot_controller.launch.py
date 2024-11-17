#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)
from webots_ros2_driver.webots_controller import WebotsController

package_name = "mh180_simulation"


def add_prefix_to_joint_names(urdf_path, prefix):
    with open(urdf_path, "r") as urdf_file:
        urdf_content = urdf_file.read()

    # modified_urdf_content = urdf_content.replace('<joint name="', f'<joint name="{prefix}')
    modified_urdf_content = urdf_content.replace('<link name="', f'<link name="{prefix}')
    modified_urdf_content = modified_urdf_content.replace('link="', f'link="{prefix}')

    modified_urdf_path = urdf_path.replace(".urdf", "_prefix.urdf")
    with open(modified_urdf_path, "w") as modified_urdf_file:
        modified_urdf_file.write(modified_urdf_content)

    return modified_urdf_path


def launch_nodes_withconfig(context, *args, **kwargs):

    package_dir = get_package_share_directory(package_name)

    # Declare launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    robot_description_path = os.path.join(package_dir, "urdf", "motoman_mh180.urdf")
    mh180_description_path = add_prefix_to_joint_names(robot_description_path, "mh180/")
    ros2_control_params = os.path.join(package_dir, "config", "ros2_controllers.yaml")
    controller_manager_timeout = ["--controller-manager-timeout", "100"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""

    controller_node = WebotsController(
        namespace="mh180",
        robot_name="motoman_mh_180_120",
        parameters=[
            {"robot_description": mh180_description_path},
            {"use_sim_time": use_sim_time},
            {"update_rate": 60},
            {"set_robot_state_publisher": False},
            {"motoman_mh180_controller.type": "joint_trajectory_controller/JointTrajectoryController"},
            {"mh180_joint_state_broadcaster.type": "joint_state_broadcaster/JointStateBroadcaster"},
            {"motoman_mh180_controller.params_file": ros2_control_params},
        ],
        respawn=True,
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["motoman_mh180_controller", "-c", "mh180/controller_manager"] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["mh180_joint_state_broadcaster", "-c", "mh180/controller_manager"] + controller_manager_timeout,
    )

    ros_control_spawners = [
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    with open(mh180_description_path) as f:
        mh180_robot_description = f.read()
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            # {"robot_description": Command(["xacro ", LaunchConfiguration("model")])},
            {"robot_description": mh180_robot_description},
            {"use_sim_time": use_sim_time},
        ],
        namespace=namespace,
    )

    default_rviz2_path = os.path.join(package_dir, "rviz/mh180_description_viz.rviz")
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d" + default_rviz2_path],
        namespace=namespace,
    )

    # Wait for the simulation to be ready to start RViz, the navigation and spawner
    waiting_nodes = WaitForControllerConnection(target_driver=controller_node, nodes_to_start=ros_control_spawners)

    return [
        controller_node,
        robot_state_publisher,
        waiting_nodes,
        rviz_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ]


def generate_launch_description():

    get_package_share_directory(package_name)

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace", default_value="mh180", description="Top-level namespace"
    )

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    declare_launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Weather to launch rviz or not"
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
