cmake_minimum_required(VERSION 3.8)
project(my_diff_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# Install package files
install(
  DIRECTORY 
    urdf/     # Your robot URDF/XACRO files
    launch/   # Launch files
    config/   # YAML config files (including diff_drive_controllers.yaml)
    meshes/   # 3D models (if any)
    worlds/   # Gazebo worlds (if any)
  DESTINATION share/${PROJECT_NAME}
)

# If you have custom C++ nodes, you'd add them here:
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp std_msgs)
# install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

# Export dependencies so other packages can use them
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  gazebo_ros
  ros2_control
  ros2_controllers
)

# Finalize the package configuration
ament_package()