# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_dir = get_package_share_directory("helix_generator")


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    # Launch the moveit control_action client node
    helix_broadcast_node = Node(
        package="helix_generator",
        executable="broadcast_helix.py",
        name="broadcast_helix",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    default_rviz2_path = os.path.join(package_dir, "rviz/broadcast_helix.rviz")
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d" + default_rviz2_path],
    )

    return [helix_broadcast_node, rviz_node]


def generate_launch_description():

    # General arguments
    declared_use_sim_time_arguments = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
    )

    declared_launch_rviz_arguments = DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Launch RViz"
    )

    ld = LaunchDescription()

    ld.add_action(declared_use_sim_time_arguments)
    ld.add_action(declared_launch_rviz_arguments)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
