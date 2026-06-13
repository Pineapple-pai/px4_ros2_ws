#!/usr/bin/env bash
set -euo pipefail
source "/home/p/px4_ros2_ws/.runtime/ros2_autonomy_env.sh"
exec ros2 run px4_obstacle_tools gz_scan_min_distance --ros-args \
  -p gz_scan_topic:=/world/room_obstacles/model/x500_lidar_front_0/link/lidar_sensor_link/sensor/lidar/scan \
  -p distance_topic:=/perception/min_obstacle_distance
