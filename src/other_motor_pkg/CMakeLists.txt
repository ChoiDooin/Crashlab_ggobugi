cmake_minimum_required(VERSION 3.8)
project(other_motor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(cap_motor_node src/cap_motor_node.cpp)
ament_target_dependencies(cap_motor_node
  rclcpp
  std_msgs
  ament_index_cpp
)
target_link_libraries(cap_motor_node pigpiod_if2)

install (TARGETS
  cap_motor_node
  )
# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
