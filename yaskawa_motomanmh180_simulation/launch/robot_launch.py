#!/usr/bin/env python

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node

PACKAGE_NAME = 'yaskawa_motomanmh180_simulation'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)

    # Starts Webots
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'heusch_mfg_plant.wbt']),
        mode="realtime",
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'mh180.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_controllers.yaml')

    # Driver nodes
    # When having multiple robot it is mandatory to specify the robot name.
    mh180_robot_driver = WebotsController(
        robot_name='motoman_mh_180_120',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            ros2_control_params
        ],
    )

    # Other ROS 2 nodes
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        #arguments=['yaskawa_mh180_controller', '-c', 'mh180/controller_manager'] + controller_manager_timeout,
        arguments=['yaskawa_mh180_controller'] + controller_manager_timeout,
        #namespace='mh180',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        #arguments=['yaskawa_joint_state_broadcaster', '-c', 'mh180/controller_manager'] + controller_manager_timeout,
        arguments=['yaskawa_joint_state_broadcaster'] + controller_manager_timeout,
        #namespace='mh180',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'},
                ],
    )

    ros_control_spawners = [trajectory_controller_spawner, joint_state_broadcaster_spawner]

    # Wait for the simulation to be ready to start RViz, the navigation and spawner
    waiting_nodes = WaitForControllerConnection(
        target_driver=mh180_robot_driver,
        nodes_to_start=ros_control_spawners
    )

    return LaunchDescription([
        webots,
        webots._supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),

        robot_state_publisher,
        mh180_robot_driver,
        waiting_nodes,

        # Kill all the nodes when the driver node is shut down
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=mh180_robot_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
