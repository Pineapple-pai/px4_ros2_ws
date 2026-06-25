#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
LIVOX_SDK2_PREFIX="${LIVOX_SDK2_PREFIX:-$HOME/.local/livox_sdk2}"

cd "${WS_DIR}"

chmod +x \
  scripts/install_livox_sdk2.sh \
  scripts/check_planning_prereqs.sh \
  scripts/start_autonomy_with_planning.sh

echo "[INFO] Step 1/3: Install Livox-SDK2"
./scripts/install_livox_sdk2.sh

echo "[INFO] Step 2/3: Build required ROS 2 packages"
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  livox_ros_driver2 \
  quadrotor_msgs \
  ego_planner \
  fast_lio \
  px4_nav2_bridge \
  px4_fastlio_bridge \
  px4_trajectory_interface \
  --cmake-args -DLIVOX_SDK2_PREFIX="${LIVOX_SDK2_PREFIX}" -DDISTRO_ROS=humble

echo "[INFO] Step 3/3: Check prerequisites"
./scripts/check_planning_prereqs.sh

echo
echo "[INFO] Bootstrap complete."
echo "[INFO] Next step:"
echo "  cd ${WS_DIR}"
echo "  export LIVOX_SDK2_PREFIX=${LIVOX_SDK2_PREFIX}"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  USE_FASTLIO=true USE_LIVOX=false ./scripts/start_autonomy_with_planning.sh"
