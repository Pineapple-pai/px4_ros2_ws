#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
LIVOX_SDK2_PREFIX="${LIVOX_SDK2_PREFIX:-$HOME/.local/livox_sdk2}"

fail() {
  echo "[FAIL] $1"
  exit 1
}

pass() {
  echo "[ OK ] $1"
}

warn() {
  echo "[WARN] $1"
}

if [[ ! -d "${WS_DIR}" ]]; then
  fail "Workspace directory not found: ${WS_DIR}"
fi

source /opt/ros/humble/setup.bash

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  fail "Workspace not built yet. Run colcon build in ${WS_DIR} first."
fi
pass "Workspace install/setup.bash exists"

required_pkgs=(
  livox_ros_driver2
  quadrotor_msgs
  ego_planner
  fast_lio
  px4_nav2_bridge
  px4_fastlio_bridge
  px4_trajectory_interface
)

for pkg in "${required_pkgs[@]}"; do
  if ! colcon list --base-paths "${WS_DIR}/src" 2>/dev/null | awk '{print $1}' | grep -qx "${pkg}"; then
    fail "Package not found by colcon: ${pkg}"
  fi
done
pass "All required source packages are discoverable by colcon"

source "${WS_DIR}/install/setup.bash"

installed_pkgs=(
  px4_nav2_bridge
  px4_fastlio_bridge
  px4_trajectory_interface
)

for pkg in "${installed_pkgs[@]}"; do
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    fail "Installed ROS package not found in environment: ${pkg}"
  fi
done
pass "Bridge packages are present in ROS environment"

if [[ -f "${LIVOX_SDK2_PREFIX}/lib/liblivox_lidar_sdk_shared.so" ]]; then
  pass "Livox-SDK2 shared library found under ${LIVOX_SDK2_PREFIX}"
else
  fail "Livox-SDK2 shared library not found under ${LIVOX_SDK2_PREFIX}. Run ./scripts/install_livox_sdk2.sh."
fi

required_files=(
  "${WS_DIR}/src/livox_ros_driver2/config/MID360_config.mid360_nuc_template.json"
  "${WS_DIR}/src/px4_fastlio_bridge/config/mid360_ego_planner.template.yaml"
  "${WS_DIR}/src/px4_trajectory_interface/launch/ego_planner_offboard.launch.py"
)

for file in "${required_files[@]}"; do
  [[ -f "${file}" ]] || fail "Required file missing: ${file}"
done
pass "Required planning config/launch files exist"

if command -v bash >/dev/null 2>&1; then
  bash -n "${WS_DIR}/scripts/start_autonomy_with_planning.sh" || fail "start_autonomy_with_planning.sh has invalid shell syntax"
  pass "start_autonomy_with_planning.sh shell syntax OK"
fi

warn "Prerequisites look good. You can continue with:"
echo "  cd ${WS_DIR}"
echo "  ./scripts/start_autonomy_with_planning.sh"
