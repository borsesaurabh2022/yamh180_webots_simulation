
import os
import pathlib
import yaml
from launch.actions import LogInfo
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes


PACKAGE_NAME = 'motoman_robotcrtl'


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory(PACKAGE_NAME)

    def load_resource_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'urdf', filename)).read_text()
    
    def load_config_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'config', filename)).read_text()

    def load_config_yaml(filename):
        return yaml.safe_load(load_config_file(filename))

    # Check if moveit is installed
    if 'moveit' in get_packages_with_prefixes():
        # Configuration
        description = {'robot_description': load_resource_file('motoman_mh180.urdf')}
        description_semantic = {'robot_description_semantic': load_config_file('motoman_mh_180_120.srdf')}
        description_kinematics = {'robot_description_kinematics': load_config_yaml('kinematics.yaml')}
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
                    description,
                    description_semantic,
                    description_kinematics,
                    description_joint_limits,
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
                    description,
                    description_semantic,
                    description_kinematics,
                    description_joint_limits,
                    moveit_controllers,
                    movegroup,
                    sim_time,
                ],
            )
        )
    else:
        launch_description_nodes.append(LogInfo(msg='"moveit" package is not installed, \
                                                please install it in order to run this demo.'))

    return LaunchDescription(launch_description_nodes)
