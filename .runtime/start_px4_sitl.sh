#!/usr/bin/env bash
set -euo pipefail
cd "/home/p/PX4-Autopilot"
export PX4_HOME_LAT=""
export PX4_HOME_LON=""
export PX4_HOME_ALT=""
export PX4_HOME_YAW=""
export GZ_SIM_RESOURCE_PATH="/home/p/PX4-Autopilot/Tools/simulation/gz/worlds:/home/p/px4_ros2_ws/sim/worlds:${GZ_SIM_RESOURCE_PATH:-}"
if [[ "room_obstacles" == "default" ]]; then
  exec make px4_sitl gz_x500_lidar_front
else
  export PX4_GZ_WORLD="room_obstacles"
  exec make px4_sitl gz_x500_lidar_front
fi
