cmake_minimum_required(VERSION 3.8)
project(my_bot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages for Gazebo Fortress
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Gazebo Fortress packages (NOT gazebo_ros)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# ROS 2 Control packages
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# Install package files
install(
  DIRECTORY 
    launch/
    config/
    description/
    worlds/
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  ros_gz_sim
  ros_gz_bridge
  ros2_control
  ros2_controllers
)

# Finalize the package
ament_package()