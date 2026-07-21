#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
LIVOX_CONFIG="${LIVOX_CONFIG:-}"
FASTLIO_CONFIG_PATH="${FASTLIO_CONFIG_PATH:-${WS_DIR}/src/px4_fastlio_bridge/config}"
FASTLIO_CONFIG_FILE="${FASTLIO_CONFIG_FILE:-}"
AIRFRAME_CALIBRATION_ID="${AIRFRAME_CALIBRATION_ID:-}"
READINESS_DURATION_S="${READINESS_DURATION_S:-8}"
STARTUP_WAIT_S="${STARTUP_WAIT_S:-8}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
PLANNER_CLOUD_TOPIC="${PLANNER_CLOUD_TOPIC:-/autonomy/cloud_registered}"
EXPECTED_MAP_FRAME="${EXPECTED_MAP_FRAME:-world}"
EGO_MAX_VEL="${EGO_MAX_VEL:-0.25}"
EGO_MAX_ACC="${EGO_MAX_ACC:-0.35}"

fail() {
  echo "REAL_AUTONOMY_ERROR: $*" >&2
  exit 2
}

[[ "${CONFIRM_REAL_BENCH:-}" == "YES" ]] || fail "set CONFIRM_REAL_BENCH=YES after removing propellers and securing the airframe"
[[ -f /opt/ros/humble/setup.bash ]] || fail "ROS 2 Humble setup is missing"
[[ -f "${WS_DIR}/install/setup.bash" ]] || fail "workspace is not built: ${WS_DIR}/install/setup.bash"
[[ -n "${LIVOX_CONFIG}" && -f "${LIVOX_CONFIG}" ]] || fail "LIVOX_CONFIG must name an existing, aircraft-specific JSON file"
[[ -n "${FASTLIO_CONFIG_FILE}" && -f "${FASTLIO_CONFIG_PATH}/${FASTLIO_CONFIG_FILE}" ]] || fail "FASTLIO_CONFIG_FILE must name an existing calibrated YAML file"
[[ -n "${AIRFRAME_CALIBRATION_ID}" ]] || fail "AIRFRAME_CALIBRATION_ID must identify the approved sensor/airframe calibration"
[[ "${LIVOX_CONFIG}" != *template* ]] || fail "refusing Livox template config; create an aircraft-specific copy"
[[ "${FASTLIO_CONFIG_FILE}" != *template* ]] || fail "refusing FAST-LIO template config; create an aircraft-specific calibrated copy"

python3 - "${LIVOX_CONFIG}" <<'PY'
import ipaddress
import hashlib
import json
import subprocess
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    config = json.load(stream)
mid360 = config.get("MID360", {})
host = mid360.get("host_net_info", {})
lidars = config.get("lidar_configs", [])
required_host_keys = ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip")
missing = [key for key in required_host_keys if not host.get(key)]
if missing or not lidars or not lidars[0].get("ip"):
    raise SystemExit(f"invalid Livox network config; missing={missing}, lidar_ip={bool(lidars and lidars[0].get('ip'))}")
for key in required_host_keys:
    ipaddress.ip_address(host[key])
ipaddress.ip_address(lidars[0]["ip"])
addresses = json.loads(subprocess.check_output(["ip", "-j", "-4", "address", "show"], text=True))
local_ips = {item["local"] for link in addresses for item in link.get("addr_info", [])}
if host["point_data_ip"] not in local_ips:
    raise SystemExit(f"Livox host IP {host['point_data_ip']} is not assigned to this computer")
digest = hashlib.sha256(open(sys.argv[1], "rb").read()).hexdigest()[:12]
print(f"LIVOX_CONFIG_OK host={host['point_data_ip']} lidar={lidars[0]['ip']} sha256={digest}")
PY

echo "FASTLIO_CONFIG_OK calibration=${AIRFRAME_CALIBRATION_ID} sha256=$(sha256sum "${FASTLIO_CONFIG_PATH}/${FASTLIO_CONFIG_FILE}" | cut -c1-12)"

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

pids=()
cleanup() {
  trap - EXIT INT TERM
  if ((${#pids[@]})); then
    kill -TERM "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch px4_fastlio_bridge fastlio_mapping.launch.py \
  use_livox:=true use_sim_time:=false rviz:=false \
  livox_config:="${LIVOX_CONFIG}" \
  fastlio_config_path:="${FASTLIO_CONFIG_PATH}" \
  fastlio_config_file:="${FASTLIO_CONFIG_FILE}" \
  publish_nav2_tf:=false &
pids+=("$!")

ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py \
  use_sim_time:=false use_sim_bridge:=false use_depth_camera_fastlio:=false \
  use_fastlio_bridge:=false use_native_3d_pointcloud:=true use_scan_fallback:=false \
  native_pointcloud_topic:="${PLANNER_CLOUD_TOPIC}" \
  planner_odom_topic:=/odom launch_rviz:="${LAUNCH_RVIZ}" \
  use_ego_planner:=true use_simple_avoidance_fallback:=false \
  trajectory_auto_arm:=false trajectory_auto_set_offboard:=true \
  require_armed_before_offboard:=true require_local_position_before_offboard:=true \
  stop_on_px4_failsafe:=true suspend_on_external_mode_command:=true \
  max_vel:="${EGO_MAX_VEL}" max_acc:="${EGO_MAX_ACC}" &
pids+=("$!")

sleep "${STARTUP_WAIT_S}"
for pid in "${pids[@]}"; do
  kill -0 "${pid}" 2>/dev/null || fail "a launch process exited during startup"
done

timeout 15s python3 "${WS_DIR}/scripts/check_real_bench_readiness.py" \
  --duration-s "${READINESS_DURATION_S}" \
  --planner-odom-topic /odom \
  --ego-local-map-topic /autonomy/local_map \
  --expected-local-map-frame "${EXPECTED_MAP_FRAME}"

echo "REAL_AUTONOMY_READY: readiness gate passed; keep props removed until all manual checklist items pass"
wait -n "${pids[@]}"
fail "a required launch process exited"