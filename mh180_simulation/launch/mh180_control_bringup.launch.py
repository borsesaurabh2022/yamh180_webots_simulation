# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_movegroup = LaunchConfiguration("launch_movegroup")
    launch_servo = LaunchConfiguration("launch_servo")
    launch_rviz = LaunchConfiguration("launch_rviz")
    pipeline_id = LaunchConfiguration("pipeline_id")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval  #

    # launch robotinobase controllers with individual namespaces

    # Launch robotinobase1 controller
    movegroup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mh180_simulation"),
                        "launch",
                        "mh180_moveit_simulation.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(launch_movegroup),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
            "pipeline_id": pipeline_id,
        }.items(),
    )

    # Launch robotinobase1 controller
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mh180_simulation"),
                        "launch",
                        "mh180_servo_simulation.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(launch_servo),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    # Launch the moveit control_action client node
    action_server = Node(
        package="mh180_simulation",
        executable="mh180_moveittrajctrl_actionserver",
        name="mh180_moveittrajctrl_actionserver",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Launch the moveit control_action client node
    helix_generator = Node(
        package="mh180_simulation",
        executable="generate_broadcast_helix",
        name="generate_broadcast_helix",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [movegroup, servo, action_server, helix_generator]


def generate_launch_description():

    # UR specific arguments
    declared_namespace_arguments = DeclareLaunchArgument(
        "namespace", default_value="mh180", description="Type/series of used UR robot."
    )

    # General arguments
    declared_use_sim_time_arguments = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
    )

    declared_launch_servo_arguments = DeclareLaunchArgument(
        "launch_servo", default_value="false", description="Launch Servo"
    )

    declared_launch_movegroup_arguments = DeclareLaunchArgument(
        "launch_movegroup", default_value="true", description="launch movegroup"
    )

    declared_launch_rviz_arguments = DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Launch RViz"
    )

    declared_pipeline_id_arguments = DeclareLaunchArgument(
        "pipeline_id",
        default_value="pilz",
        description="Configure the planning pipeline for MoveIt. Default is pilz.",
    )

    ld = LaunchDescription()

    ld.add_action(declared_namespace_arguments)
    ld.add_action(declared_use_sim_time_arguments)
    ld.add_action(declared_launch_servo_arguments)
    ld.add_action(declared_launch_movegroup_arguments)
    ld.add_action(declared_launch_rviz_arguments)
    ld.add_action(declared_pipeline_id_arguments)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
