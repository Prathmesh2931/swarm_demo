cmake_minimum_required(VERSION 3.8)
project(waypoint_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavToWaypoint.action"
  "srv/GetWaypoints.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    action_msgs
    builtin_interfaces
)

# Install action and service files
install(
  FILES
    action/NavToWaypoint.action
  DESTINATION
    share/${PROJECT_NAME}/action
)

install(
  FILES
    srv/GetWaypoints.srv
  DESTINATION
    share/${PROJECT_NAME}/srv
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

