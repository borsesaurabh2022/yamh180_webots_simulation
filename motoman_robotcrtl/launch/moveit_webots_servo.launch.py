
import os
import pathlib
import yaml
import launch_ros
from launch.actions import LogInfo
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    
    launch_description_nodes = []
    package_dir = get_package_share_directory('motoman_robotcrtl')

    def load_resource_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'urdf', filename)).read_text()
    
    def load_config_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'config', filename)).read_text()

    def load_config_yaml(filename):
        return yaml.safe_load(load_config_file(filename))
    
    moveit_config = (
        MoveItConfigsBuilder("motoman_mh_180_120", package_name="motoman_robotcrtl")
        .robot_description(file_path="config/motoman_mh_180_120.urdf.xacro")
        .to_moveit_configs()
    )
    
    # Launch Servo as a standalone node or as a "node component" for better latency/efficiency
    launch_as_standalone_node = LaunchConfiguration("launch_as_standalone_node", default="false")

        # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("motoman_robotcrtl")
        .yaml("config/motoman_simulated_config.yaml")
        .to_dict()
    }
    
    # This filter parameter should be >1. Increase it for greater smoothing but slower motion.
    low_pass_filter_coeff = {"butterworth_filter_coeff": 1.5}
    
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
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    low_pass_filter_coeff,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )
    
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            low_pass_filter_coeff,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            #{'use_intra_process_comms' : True}
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    # Check if moveit is installed
    if 'moveit' in get_packages_with_prefixes():
        # Configuration
        #description = {'robot_description': load_resource_file('motoman_mh180.urdf')}
        #description_semantic = {'robot_description_semantic': load_config_file('motoman_mh_180_120.srdf')}
        #description_kinematics = {'robot_description_kinematics': load_config_yaml('kinematics.yaml')}
        description_joint_limits = {'robot_description_joint_limits': load_config_yaml('joint_limits.yaml')}
        sim_time = {'use_sim_time': True}

        # Rviz node
        rviz_config_file = os.path.join(package_dir, 'rviz', 'moveit.rviz')

        launch_description_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    #description,
                    #description_semantic,
                    #description_kinematics,
                    #description_joint_limits,
                    sim_time
                ],
            )
        )
        
        # MoveIt2 node
        movegroup = {'move_group': load_config_yaml('moveit_movegroup.yaml')}
        moveit_controllers = {
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager': load_config_yaml('moveit_controllers_mod.yaml')
        }

        launch_description_nodes.append(
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    #description,
                    #description_semantic,
                    #description_kinematics,
                    #description_joint_limits,
                    moveit_controllers,
                    movegroup,
                    sim_time,
                ],
            )
        )
    else:
        launch_description_nodes.append(LogInfo(msg='"moveit" package is not installed, \
                                                please install it in order to run this demo.'))
        
    launch_description_nodes.append(servo_node)
    launch_description_nodes.append(container)

    return LaunchDescription(launch_description_nodes)

