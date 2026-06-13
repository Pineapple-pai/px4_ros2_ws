#!/usr/bin/env bash
set -euo pipefail

PX4_STATUS_LOG="/home/p/px4_ros2_ws/.runtime/logs/px4.log"
echo "Waiting for ROS2 external mode registration..."
until grep -q "Got RegisterExtComponentReply" "${PX4_STATUS_LOG}" || grep -q "Registering 'ROS2 Autonomy'" "/home/p/px4_ros2_ws/.runtime/logs/ros2.log"; do
  sleep 1
done
sleep 2
exec "/home/p/px4_ros2_ws/.runtime/start_qgc.sh"
