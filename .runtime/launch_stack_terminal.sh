#!/usr/bin/env bash
set -euo pipefail

LOG_DIR="/home/p/px4_ros2_ws/.runtime/logs"
rm -f "${LOG_DIR}/px4.log" "${LOG_DIR}/agent.log" "${LOG_DIR}/ros2.log" "${LOG_DIR}/qgc.log"
touch "${LOG_DIR}/px4.log" "${LOG_DIR}/agent.log" "${LOG_DIR}/ros2.log" "${LOG_DIR}/qgc.log"

cleanup() {
  pkill -P 4771 || true
}
trap cleanup EXIT INT TERM

echo "[stack] starting PX4 SITL..."
"/home/p/px4_ros2_ws/.runtime/start_px4_sitl.sh" >"${LOG_DIR}/px4.log" 2>&1 &
PX4_PID=$!

sleep 2
echo "[stack] starting MicroXRCEAgent..."
"/home/p/px4_ros2_ws/.runtime/start_agent.sh" >"${LOG_DIR}/agent.log" 2>&1 &
AGENT_PID=$!

sleep 2
echo "[stack] starting QGroundControl..."
"/home/p/px4_ros2_ws/.runtime/wait_for_qgc_start.sh" >"${LOG_DIR}/qgc.log" 2>&1 &
QGC_PID=$!

sleep 2
echo "[stack] waiting for /fmu and starting ROS2 autonomy..."
"/home/p/px4_ros2_ws/.runtime/wait_for_fmu.sh" >"${LOG_DIR}/ros2.log" 2>&1 &
ROS2_PID=$!

GZ_SCAN_PID=""
if [[ "true" == "true" ]]; then
  echo "[stack] starting Gazebo scan distance bridge..."
  "/home/p/px4_ros2_ws/.runtime/start_gz_scan_distance.sh" >"${LOG_DIR}/gz_scan_distance.log" 2>&1 &
  GZ_SCAN_PID=$!
fi

echo "[stack] pids: PX4=${PX4_PID} Agent=${AGENT_PID} ROS2=${ROS2_PID} QGC=${QGC_PID} GzScan=${GZ_SCAN_PID:-disabled}"
echo "[stack] logs: ${LOG_DIR}/px4.log, ${LOG_DIR}/agent.log, ${LOG_DIR}/ros2.log, ${LOG_DIR}/qgc.log, ${LOG_DIR}/gz_scan_distance.log"
echo
echo "==== PX4 SITL ===="
tail -n 0 -F "${LOG_DIR}/px4.log" &
TAIL_PX4=$!
echo "==== MicroXRCEAgent ===="
tail -n 0 -F "${LOG_DIR}/agent.log" &
TAIL_AGENT=$!
echo "==== ROS2 Autonomy ===="
tail -n 0 -F "${LOG_DIR}/ros2.log" &
TAIL_ROS2=$!
echo "==== QGroundControl launcher ===="
tail -n 0 -F "${LOG_DIR}/qgc.log" &
TAIL_QGC=$!
if [[ "true" == "true" ]]; then
  echo "==== Gazebo Scan Distance ===="
  tail -n 0 -F "${LOG_DIR}/gz_scan_distance.log" &
  TAIL_GZ_SCAN=$!
fi

wait ${PX4_PID} ${AGENT_PID} ${ROS2_PID} ${QGC_PID}
