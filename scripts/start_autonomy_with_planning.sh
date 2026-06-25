#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
TERMINAL_LAYOUT="${TERMINAL_LAYOUT:-windows}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
USE_LIVOX="${USE_LIVOX:-false}"
USE_FASTLIO="${USE_FASTLIO:-true}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}"
USE_SIM_EGO_BRIDGE="${USE_SIM_EGO_BRIDGE:-true}"
USE_DEPTH_CAMERA_FASTLIO="${USE_DEPTH_CAMERA_FASTLIO:-false}"

PX4_MODEL="${PX4_MODEL:-iris_rplidar}"
PX4_WORLD="${PX4_WORLD:-room_obstacles}"
MISSION_ALTITUDE_M="${MISSION_ALTITUDE_M:-1.5}"

LIVOX_CONFIG="${LIVOX_CONFIG:-${WS_DIR}/src/livox_ros_driver2/config/MID360_config.mid360_nuc_template.json}"
FASTLIO_CONFIG_PATH="${FASTLIO_CONFIG_PATH:-${WS_DIR}/src/px4_fastlio_bridge/config}"
FASTLIO_CONFIG_FILE="${FASTLIO_CONFIG_FILE:-mid360_ego_planner.template.yaml}"

EGO_MAX_VEL="${EGO_MAX_VEL:-2.5}"
EGO_MAX_ACC="${EGO_MAX_ACC:-3.0}"
EGO_PLANNING_HORIZON="${EGO_PLANNING_HORIZON:-7.5}"
EGO_MAP_SIZE_X="${EGO_MAP_SIZE_X:-30.0}"
EGO_MAP_SIZE_Y="${EGO_MAP_SIZE_Y:-30.0}"
EGO_MAP_SIZE_Z="${EGO_MAP_SIZE_Z:-4.0}"
EGO_OBSTACLE_INFLATION="${EGO_OBSTACLE_INFLATION:-0.25}"

if [[ "${USE_DEPTH_CAMERA_FASTLIO}" == "true" && "${PX4_MODEL}" == "iris_rplidar" ]]; then
  PX4_MODEL="iris_depth_camera"
fi

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built yet: ${WS_DIR}/install/setup.bash"
  exit 1
fi

if [[ "${TERMINAL_LAYOUT}" == "headless" || "${TERMINAL_LAYOUT}" == "tmux" ]]; then
  echo "start_autonomy_with_planning.sh is configured for GUI validation."
  echo "Use TERMINAL_LAYOUT=windows or TERMINAL_LAYOUT=tabs."
  echo "Current TERMINAL_LAYOUT=${TERMINAL_LAYOUT}"
  exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found, cannot open Gazebo/QGC GUI windows."
  echo "Install gnome-terminal or run the lower-level scripts manually."
  exit 1
fi

if [[ -x "${WS_DIR}/scripts/stop_px4_sim.sh" ]]; then
  "${WS_DIR}/scripts/stop_px4_sim.sh" >/dev/null 2>&1 || true
fi

if [[ ! -x "${WS_DIR}/scripts/check_planning_prereqs.sh" ]]; then
  echo "Missing prerequisite checker: ${WS_DIR}/scripts/check_planning_prereqs.sh"
  exit 1
fi

"${WS_DIR}/scripts/check_planning_prereqs.sh"

export WS_DIR TERMINAL_LAYOUT USE_SIM_TIME USE_LIVOX USE_FASTLIO FASTLIO_RVIZ
export PX4_MODEL PX4_WORLD MISSION_ALTITUDE_M LIVOX_CONFIG FASTLIO_CONFIG_PATH FASTLIO_CONFIG_FILE

"${WS_DIR}/scripts/start_px4_sim.sh" &
sleep 8

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py \
  use_sim_time:="${USE_SIM_TIME}" \
  use_fastlio_bridge:="${USE_FASTLIO}" \
  use_sim_bridge:="${USE_SIM_EGO_BRIDGE}" \
  use_depth_camera_fastlio:="${USE_DEPTH_CAMERA_FASTLIO}" \
  fixed_goal_altitude_m:="${MISSION_ALTITUDE_M}" \
  max_vel:="${EGO_MAX_VEL}" \
  max_acc:="${EGO_MAX_ACC}" \
  planning_horizon:="${EGO_PLANNING_HORIZON}" \
  map_size_x:="${EGO_MAP_SIZE_X}" \
  map_size_y:="${EGO_MAP_SIZE_Y}" \
  map_size_z:="${EGO_MAP_SIZE_Z}" \
  obstacle_inflation:="${EGO_OBSTACLE_INFLATION}"
