cmake_minimum_required(VERSION 3.8)
project(px4_autonomy_mode)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})

add_executable(px4_autonomy_mode
  src/main.cpp
)

target_link_libraries(px4_autonomy_mode
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)

ament_target_dependencies(px4_autonomy_mode
  std_msgs
  geometry_msgs
)

install(TARGETS px4_autonomy_mode
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
