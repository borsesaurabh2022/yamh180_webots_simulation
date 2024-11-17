#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
import os
import pathlib

import yaml
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_name = "mh180_simulation"


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory(package_name)
    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    LaunchConfiguration("pipeline_id")

    nodes_to_start = []

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval  #

    package_dir = get_package_share_directory(launch_configuration["namespace"] + "_simulation")

    def load_config(filename):
        return pathlib.Path(os.path.join(package_dir, "config", filename)).read_text()

    def load_yaml(filename):
        return yaml.safe_load(load_config(filename))

    # Check if moveit is installed
    if "moveit" in get_packages_with_prefixes():
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

        if launch_configuration["pipeline_id"] == "ompl":
            movegroup = {"move_group": load_yaml("ompl_planning.yaml")}

        elif launch_configuration["pipeline_id"] == "pilz":
            movegroup = {"move_group": load_yaml("pilz_industrial_motion_planner_planning.yaml")}

        pilz_cartesian_limits = {"robot_description_planning": load_yaml("pilz_cartesian_limits.yaml")}

        # Trajectory Execution Configuration
        trajectory_execution = {
            "moveit_manage_controllers": False,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
        }

        planning_scene_monitor_parameters = {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }

        # MoveIt2 node
        moveit_controllers = {
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": load_yaml("moveit_controllers.yaml"),
        }

        # Start the actual move_group node/action server
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            namespace=namespace,
            parameters=[
                description,
                description_semantic,
                description_kinematics,
                description_joint_limits,
                trajectory_execution,
                moveit_controllers,
                movegroup,
                pilz_cartesian_limits,
                planning_scene_monitor_parameters,
                sim_time,
            ],
        )

        # rviz with moveit configuration
        rviz_config_file = os.path.join(package_dir, "rviz", "moveit_visualization.rviz")

        rviz_node = Node(
            package="rviz2",
            condition=IfCondition(launch_rviz),
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                description,
                description_semantic,
                description_kinematics,
                description_joint_limits,
                sim_time,
            ],
            namespace=namespace,
        )

        nodes_to_start = [
            LogInfo(msg=f"Planning configuration: {movegroup}"),
            move_group_node,
            rviz_node,
        ]

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

    declared_arguments.append(
        DeclareLaunchArgument(
            "pipeline_id",
            default_value="pilz",
            description="Configure the planning pipeline for MoveIt. Default is pilz.",
        )
    )

    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
