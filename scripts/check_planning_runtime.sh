#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"

pass() {
  echo "[ OK ] $1"
}

warn() {
  echo "[WARN] $1"
}

fail() {
  echo "[FAIL] $1"
  exit 1
}

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

if ! timeout 5 ros2 topic list >/dev/null 2>&1; then
  fail "ROS 2 graph is not reachable. Make sure the autonomy stack is running in another terminal."
fi

required_nodes=(
  "/qgc_ego_goal_bridge"
  "/trajectory_interface"
  "/ego_planner_node"
  "/ego_traj_server"
  "/lio_odometry_bridge"
)

required_topics=(
  "/autonomy/lio_odometry"
  "/autonomy/local_map"
  "/move_base_simple/goal"
  "/planning/bspline"
  "/planning/position_cmd"
  "/fmu/in/offboard_control_mode"
  "/fmu/in/trajectory_setpoint"
  "/fmu/out/vehicle_status_v4"
)

node_list="$(ros2 node list 2>/dev/null || true)"
topic_list="$(ros2 topic list 2>/dev/null || true)"

for node in "${required_nodes[@]}"; do
  if grep -qx "${node}" <<< "${node_list}"; then
    pass "Node online: ${node}"
  else
    warn "Node not found yet: ${node}"
  fi
done

for topic in "${required_topics[@]}"; do
  if grep -qx "${topic}" <<< "${topic_list}"; then
    pass "Topic online: ${topic}"
  else
    warn "Topic not found yet: ${topic}"
  fi
done

echo
echo "[INFO] Suggested next checks:"
echo "  ros2 topic echo /move_base_simple/goal --once"
echo "  ros2 topic hz /planning/position_cmd"
echo "  ros2 topic echo /fmu/in/trajectory_setpoint --once"
echo "  ros2 topic echo /fmu/out/vehicle_status_v4 --once"
