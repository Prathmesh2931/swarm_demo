cmake_minimum_required(VERSION 3.8)
project(swift_pico)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(swift_msgs REQUIRED)
find_package(controller_msg REQUIRED)
find_package(error_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(waypoint_navigation REQUIRED)
find_package(OpenCV REQUIRED)
find_package(swift_pico_description REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

#Target installations
#builds cpp source code into an executable
add_executable(pico_controller_PID_cpp src/pico_controller_PID.cpp)

# Correctly links executable against the necessary libraries
ament_target_dependencies(pico_controller_PID_cpp
	rclcpp
	geometry_msgs
	swift_msgs
	controller_msg
  error_msg
)

# Task 2A: Waypoint navigation executables
# Waypoint service server
add_executable(waypoint_service_cpp src/waypoint_service.cpp)
ament_target_dependencies(waypoint_service_cpp
	rclcpp
	waypoint_navigation
)

# Waypoint action server
add_executable(pico_server_cpp src/pico_server.cpp)
ament_target_dependencies(pico_server_cpp
	rclcpp
	rclcpp_action
	geometry_msgs
	nav_msgs
	swift_msgs
	error_msg
	waypoint_navigation
	tf2
	tf2_ros
)

# Waypoint action client
add_executable(pico_client_cpp src/pico_client.cpp)
ament_target_dependencies(pico_client_cpp
	rclcpp
	rclcpp_action
	waypoint_navigation
)

#task1
#install the compiled target for 'ros2 run' to find it.
install(TARGETS
	pico_controller_PID_cpp
	waypoint_service_cpp
	pico_server_cpp
	pico_client_cpp
	DESTINATION lib/${PROJECT_NAME}
)

#Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

#Install scripts
install(PROGRAMS
  scripts/task1a.cpp
  scripts/task1a.py
  src/pico_controller_PID.py
  src/pico_controller_LQR.py
  #src/pico_controller_PID.cpp
  src/pico_controller_LQR.cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
