cmake_minimum_required(VERSION 3.8)
project(helix_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mh180_interface REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    helix_generator/broadcast_helix.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other package resources
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
