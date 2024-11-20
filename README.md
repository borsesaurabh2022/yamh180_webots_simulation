## yaskawa_mh180_ros2_control
 This is a ROS2 package to simulate the industrial robot YASKAWA MOTOMAN MH180 in webots simulation world and path planning unsing moveit interface.
 the URDF description of robot is obtained from [motoman_industrail] (https://github.com/ros-industrial/motoman_experimental) more information about the technical specification of the robot can be refered from https://www.yaskawa.fr/yaskawa.fr/Robots%20d%27occasion/Brochures/Flyer_Robot_MH180_E_03.2017.pdf

![Screenshot from 2023-08-09 23-52-24](https://github.com/borsesaurabh2022/yaskawa_mh180_ros2_control/assets/103029292/1a6101eb-f836-4ac1-80f4-4079a5fe1ba9)

## Prerequisites
  - Tested for ubuntu 24.04
  - ROS 2 Jazzy
  - Webots R2023b
  - Webots ROS 2 interface

## source ros2 environment and create a workspace
  ### Source ROS2
    . /opt/ros/$ROS_DISTRO/setup.bash

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

  ### webots_ros2
    git clone https://git.fh-aachen.de/sb8431s/wirksam_moveit2_tutorial.git

  ### Build everything
    cd ~/ros2_ws/src
    colcon build --symlink-install
    source install/setup.bash

## launch webot simulation world and moveit planner
  ### command to launch webots simulation, controller and spawn the robot

      ros2 launch mh180_simulation mh180_simulation.launch.py launch_rviz:=false

  ### command to launch moveit planner

      ros2 launch mh180_simulation mh180_control_bringup.launch.py launch_srvo:=true
