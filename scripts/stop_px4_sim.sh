#!/usr/bin/env bash
set -euo pipefail

pkill -f "px4_sitl_default" || true
pkill -f "MicroXRCEAgent udp4 -p" || true
pkill -f "/home/p/下载/PX4/qgc-daily-root/squashfs-root/AppRun" || true
pkill -f "QGroundControl Daily" || true
pkill -f "ros2 launch px4_autonomy_bringup autonomy_stack.launch.py" || true
pkill -f "px4_autonomy_mode" || true
pkill -f "gz_scan_min_distance" || true
pkill -f "ros_gz_bridge parameter_bridge" || true
pkill -f "gz sim" || true

echo "Requested shutdown for PX4 SITL, Gazebo, MicroXRCEAgent, QGroundControl, ROS2 bridge, and ROS2 autonomy."
