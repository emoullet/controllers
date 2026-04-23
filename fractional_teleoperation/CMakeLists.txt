cmake_minimum_required(VERSION 3.8)
project(fractional_teleoperation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(extender_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(fractional_teleoperation_core STATIC
  src/fractional_teleoperation_core.cpp
)
set_target_properties(fractional_teleoperation_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(fractional_teleoperation_core
  Eigen3
)

add_library(fractional_teleoperation_ramp STATIC
  src/ramp_profile.cpp
)
set_target_properties(fractional_teleoperation_ramp PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(fractional_teleoperation_ramp)

add_library(${PROJECT_NAME} SHARED
  src/fractional_teleoperation_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  geometry_msgs
  visualization_msgs
  robot_interfaces
  Eigen3
  extender_msgs
)
target_link_libraries(${PROJECT_NAME}
  fractional_teleoperation_core
  fractional_teleoperation_ramp
)

# Add executable for fractional teleoperation node
add_executable(fractional_teleoperation_node src/fractional_teleoperation_node.cpp)
ament_target_dependencies(fractional_teleoperation_node
  rclcpp
  geometry_msgs
  visualization_msgs
  Eigen3
  extender_msgs
)
target_link_libraries(fractional_teleoperation_node
  fractional_teleoperation_core
  fractional_teleoperation_ramp
)

# Pluginlib export for controller discovery

# Install the library
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the node executable
install(TARGETS
  fractional_teleoperation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the plugin XML file
install(FILES fractional_teleoperation_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export the plugin description
pluginlib_export_plugin_description_file(controller_interface fractional_teleoperation_controller.xml)

# Install the entire config directory (containing controllers.yaml)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})

ament_package()
