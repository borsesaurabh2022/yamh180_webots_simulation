#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher


package_name = "mh180_simulation"


def launch_nodes_withconfig(context, *args, **kwargs):

    package_dir = get_package_share_directory(package_name)

    # Declare launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "world", "mh180.wbt"]),
        mode="realtime",
        ros2_supervisor=True,
    )

    load_nodes = GroupAction(
        actions=[
            # Launch robotinobase1 controller
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("mh180_simulation"),
                                "launch",
                                "mh180_robot_controller.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "launch_rviz": launch_rviz,
                }.items(),
            )
        ]
    )

    return [
        webots,
        webots._supervisor,
        RegisterEventHandler(
            OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        load_nodes,
    ]


def generate_launch_description():

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
