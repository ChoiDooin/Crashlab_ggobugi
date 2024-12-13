cmake_minimum_required(VERSION 3.8)
project(nav_motor_pkg)

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
# CMake prefix path 설정
set(CMAKE_PREFIX_PATH "/opt/ros/humble/share/serial_driver/cmake/" ${CMAKE_PREFIX_PATH})

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
find_package(sensor_msgs REQUIRED)

# Util 라이브러리 추가
add_library(motor_util src/motor_util.cpp)
target_include_directories(motor_util PUBLIC include)
ament_target_dependencies(motor_util
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(motor_util pigpiod_if2)

##################### 노드 추가 #####################

# motor_run_node
add_executable(motor_run_node src/motor_run.cpp)
target_link_libraries(motor_run_node
  motor_util
  pigpiod_if2
)
ament_target_dependencies(motor_run_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# test_running_node
add_executable(test_running_node src/test_running_node.cpp)
ament_target_dependencies(test_running_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(test_running_node
  motor_util
  )
target_include_directories(test_running_node PRIVATE include)

#Pid gain node
add_executable(pid_gain_publisher src/pid_gain_publisher.cpp)

ament_target_dependencies(pid_gain_publisher
  rclcpp
  std_msgs
)


# # imu_publisher
# add_executable(imu_publisher src/imu_publisher.cpp)
# ament_target_dependencies(imu_publisher
#   rclcpp
#   sensor_msgs
#   serial_driver
# )


# # Install other resources
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

# Install targets
install(TARGETS
  motor_util
  motor_run_node
  test_running_node
  pid_gain_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
