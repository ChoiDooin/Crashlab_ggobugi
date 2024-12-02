cmake_minimum_required(VERSION 3.5)
project(crash_motor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# motor_test_utils 라이브러리
add_library(motor_test_utils
  src/motor_test_util.cpp
)

target_include_directories(motor_test_utils PUBLIC include)

ament_target_dependencies(motor_test_utils
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

target_link_libraries(motor_test_utils pigpiod_if2)

# Teleop Node
add_executable(teleop_node src/teleop_node.cpp)

ament_target_dependencies(teleop_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Motor Test Node
add_executable(motor_test_node src/motor_test_node.cpp)

target_link_libraries(motor_test_node
  motor_test_utils
  pigpiod_if2
)

ament_target_dependencies(motor_test_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# PID Check Node
add_executable(pid_check_node src/pid_check_node.cpp)

target_link_libraries(pid_check_node
  motor_test_utils
  pigpiod_if2
)

ament_target_dependencies(pid_check_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

#Pid gain node
add_executable(pid_gain_publisher src/pid_gain_publisher.cpp)

ament_target_dependencies(pid_gain_publisher
  rclcpp
  std_msgs
)

# Install targets
install(TARGETS
  motor_test_utils
  teleop_node
  motor_test_node
  pid_check_node
  pid_gain_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
