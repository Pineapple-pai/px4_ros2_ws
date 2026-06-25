#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
OUT_DIR="${OUT_DIR:-${WS_DIR}/log/planning_diagnostics}"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT_DIR="${OUT_DIR}/${STAMP}"

mkdir -p "${REPORT_DIR}"

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

echo "[INFO] Writing diagnostics to ${REPORT_DIR}"

run_capture() {
  local name="$1"
  shift
  {
    echo "### COMMAND: $*"
    echo
    "$@"
  } > "${REPORT_DIR}/${name}.txt" 2>&1 || true
}

run_capture node_list ros2 node list
run_capture topic_list ros2 topic list
run_capture topic_types ros2 topic list -t

run_capture goal_once timeout 5 ros2 topic echo /move_base_simple/goal --once
run_capture lio_odom_once timeout 5 ros2 topic echo /autonomy/lio_odometry --once
run_capture local_map_once timeout 5 ros2 topic echo /autonomy/local_map --once
run_capture bspline_once timeout 5 ros2 topic echo /planning/bspline --once
run_capture position_cmd_once timeout 5 ros2 topic echo /planning/position_cmd --once
run_capture offboard_mode_once timeout 5 ros2 topic echo /fmu/in/offboard_control_mode --once
run_capture trajectory_setpoint_once timeout 5 ros2 topic echo /fmu/in/trajectory_setpoint --once
run_capture vehicle_status_once timeout 5 ros2 topic echo /fmu/out/vehicle_status_v4 --once
run_capture vehicle_command_once timeout 5 ros2 topic echo /fmu/out/vehicle_command --once

run_capture position_cmd_hz timeout 8 ros2 topic hz /planning/position_cmd
run_capture lio_odom_hz timeout 8 ros2 topic hz /autonomy/lio_odometry

cat > "${REPORT_DIR}/README.txt" <<EOF
Planning diagnostics collected at: ${STAMP}

Useful files:
- node_list.txt
- topic_list.txt
- topic_types.txt
- goal_once.txt
- lio_odom_once.txt
- local_map_once.txt
- bspline_once.txt
- position_cmd_once.txt
- offboard_mode_once.txt
- trajectory_setpoint_once.txt
- vehicle_status_once.txt
- vehicle_command_once.txt
- position_cmd_hz.txt
- lio_odom_hz.txt
EOF

echo "[INFO] Diagnostics complete."
echo "[INFO] Report directory: ${REPORT_DIR}"
echo "[INFO] If you need help debugging, share the files in this directory."
