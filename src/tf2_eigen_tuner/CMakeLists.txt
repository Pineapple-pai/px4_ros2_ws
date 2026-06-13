cmake_minimum_required(VERSION 3.8)
project(tf2_eigen_tuner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 1. 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED) # 查找 Eigen3

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED) # 必须引入此包

# 2. 添加可执行文件
add_executable(multi_tuner_node src/multi_tuner_node.cpp)

add_executable(point_publisher_node src/point_publisher_node.cpp)

add_executable(axes_publisher_node src/axes_publisher_node.cpp)

target_include_directories(multi_tuner_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

# 3. 包含头文件与链接库 (推荐使用 ROS2 标准的 ament_target_dependencies)
target_include_directories(multi_tuner_node PRIVATE ${Eigen3_INCLUDE_DIRS})

target_include_directories(axes_publisher_node PRIVATE ${EIGEN3_INCLUDE_DIRS})


# 使用这个宏替换原来的 target_link_libraries，它能完美自动解析 geometry_msgs 的目标
ament_target_dependencies(multi_tuner_node
  rclcpp
  tf2_ros
  geometry_msgs
)

ament_target_dependencies(point_publisher_node
  rclcpp
  visualization_msgs
)

ament_target_dependencies(axes_publisher_node
  rclcpp
  visualization_msgs
)

# 4. 安装设置
install(TARGETS
  multi_tuner_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. 设置安装路径
install(TARGETS
  point_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  axes_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()






