cmake_minimum_required(VERSION 3.5)
project(my_bot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# === Controller Library ===
add_library(my_controller SHARED
  src/my_controller.cpp
)

target_include_directories(my_controller
  PRIVATE
  include
)

ament_target_dependencies(my_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export pluginlib description file
pluginlib_export_plugin_description_file(controller_interface my_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS my_controller
  EXPORT export_my_controller
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# === Tests (optional) ===
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_my_controller
    test/test_load_my_controller.cpp
  )

  target_link_libraries(test_load_my_controller my_controller)
  ament_target_dependencies(test_load_my_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_libraries(my_controller)
ament_package()
