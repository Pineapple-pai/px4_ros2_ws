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

if ! grep -q "TRAJECTORY_STATUS_COMPLETED" "${WS_DIR}/src/ego_planner_swarm_ros2_upstream/src/planner/plan_manage/src/traj_server.cpp"; then
  fail "ego_planner traj_server does not appear to emit TRAJECTORY_STATUS_COMPLETED."
fi
pass "ego_planner traj_server completion handoff logic present"

if ! grep -q "_control_suspended" "${WS_DIR}/src/px4_trajectory_interface/src/trajectory_interface.cpp"; then
  fail "trajectory_interface does not appear to suspend Offboard control after completion."
fi

if ! grep -q "TRAJECTORY_STATUS_COMPLETED" "${WS_DIR}/src/px4_trajectory_interface/src/trajectory_interface.cpp"; then
  fail "trajectory_interface does not appear to handle TRAJECTORY_STATUS_COMPLETED."
fi
pass "trajectory_interface completion handoff logic present"

if grep -q "mode_request_hold_s" "${WS_DIR}/src/px4_nav2_bridge/px4_nav2_bridge/qgc_reposition_goal_bridge.py"; then
  fail "qgc_reposition_goal_bridge still contains the removed mode_request_hold_s logic."
fi
pass "qgc_reposition_goal_bridge release logic present"

if ! grep -q "auto_rtl_after_finish" "${WS_DIR}/src/px4_autonomy_mode/include/px4_autonomy_mode/autonomy_mode.hpp"; then
  fail "px4_autonomy_mode missing auto_rtl_after_finish support."
fi

if ! grep -q "Switch to RTL from QGC to recover the vehicle" "${WS_DIR}/src/px4_autonomy_mode/include/px4_autonomy_mode/autonomy_mode.hpp"; then
  fail "px4_autonomy_mode does not advertise QGC RTL recovery when auto_rtl_after_finish is enabled."
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
