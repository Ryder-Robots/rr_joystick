cmake_minimum_required(VERSION 3.20)
project(rr_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rr_common_base REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/rr_joystick_node.cpp)
  
ament_target_dependencies(${PROJECT_NAME}_node
  rr_common_base
  rr_interfaces
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "rrobot::rr_joystick::RRJoystickNode"
  EXECUTABLE "rr_joystick_pub"
)
install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# ament_export_targets(${PROJECT_NAME}_node HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
