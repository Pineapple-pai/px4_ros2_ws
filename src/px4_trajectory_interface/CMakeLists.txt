cmake_minimum_required(VERSION 3.8)
project(px4_trajectory_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(trajectory_interface
  src/trajectory_interface.cpp
)

add_executable(simple_avoidance_position_cmd
  src/simple_avoidance_position_cmd.cpp
)

target_include_directories(trajectory_interface PRIVATE include)
target_include_directories(simple_avoidance_position_cmd PRIVATE include)

target_link_libraries(trajectory_interface
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)

target_link_libraries(simple_avoidance_position_cmd
  rclcpp::rclcpp
)

ament_target_dependencies(trajectory_interface
  px4_msgs
  quadrotor_msgs
)

ament_target_dependencies(simple_avoidance_position_cmd
  geometry_msgs
  nav_msgs
  quadrotor_msgs
)

install(TARGETS trajectory_interface simple_avoidance_position_cmd
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
