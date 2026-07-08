cmake_minimum_required(VERSION 3.8)
project(px4_gazebo_depth_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gazebo REQUIRED)

add_executable(gazebo_depth_to_pointcloud src/gazebo_depth_to_pointcloud.cpp)
target_include_directories(gazebo_depth_to_pointcloud PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(gazebo_depth_to_pointcloud PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(gazebo_depth_to_pointcloud ${GAZEBO_LIBRARIES} tbb)
ament_target_dependencies(gazebo_depth_to_pointcloud rclcpp sensor_msgs std_msgs px4_msgs)

install(TARGETS gazebo_depth_to_pointcloud
  DESTINATION lib/${PROJECT_NAME})

ament_package()
