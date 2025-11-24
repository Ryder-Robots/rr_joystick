cmake_minimum_required(VERSION 3.20)
project(rr_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rr_common_base REQUIRED)
find_package(rr_common_plugins REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
#find_package(gmock REQUIRED)
find_package(gtest REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(rr_joystick src/rr_joystick.cpp)
add_library(rr_joystick::rr_joystick ALIAS rr_joystick)
target_compile_features(rr_joystick PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rr_joystick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  rr_joystick PUBLIC
  ${rr_common_base_TARGETS}
  ${rr_common_plugins_TARGETS}
  ${rr_interfaces_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${gmock_TARGETS}
  ${gtest_TARGETS}
  ${rclcpp_components_TARGETS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rr_joystick PRIVATE "RR_JOYSTICK_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS rr_joystick
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(rr_joystick_node src/rr_joystick_node.cpp)
target_include_directories(rr_joystick_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rr_joystick_node rr_joystick)

install(TARGETS rr_joystick_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  rr_joystick
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
