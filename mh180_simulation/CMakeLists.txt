cmake_minimum_required(VERSION 3.8)
project(mh180_simulation)

# Add warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(mh180_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    mh180_simulation/mh180_movegroup_actioncli.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other package resources
install(
  DIRECTORY launch rviz protos world config urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
