#!/usr/bin/env bash
set -euo pipefail
source "/home/p/px4_ros2_ws/.runtime/ros2_autonomy_env.sh"
echo "Waiting for /fmu topics..."
until ros2 topic list 2>/dev/null | grep -q '^/fmu/'; do
  sleep 1
done
echo "/fmu topics detected."
sleep 1
exec "/home/p/px4_ros2_ws/.runtime/start_ros2_autonomy.sh"
