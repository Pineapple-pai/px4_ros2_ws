#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-/home/p/PX4-Autopilot}"
WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
QGC_DIR="${QGC_DIR:-/home/p/下载/PX4/qgc-daily-root/squashfs-root}"
AGENT_PORT="${AGENT_PORT:-8888}"
PX4_MODEL="${PX4_MODEL:-gz_x500}"
PX4_WORLD="${PX4_WORLD:-room_obstacles}"
PX4_HOME_LAT="${PX4_HOME_LAT:-}"
PX4_HOME_LON="${PX4_HOME_LON:-}"
PX4_HOME_ALT="${PX4_HOME_ALT:-}"
PX4_HOME_YAW="${PX4_HOME_YAW:-}"
USE_SYSTEM_GEOLOCATION="${USE_SYSTEM_GEOLOCATION:-false}"

MISSION_SIZE_M="${MISSION_SIZE_M:-2.5}"
MISSION_ALTITUDE_M="${MISSION_ALTITUDE_M:-2.0}"
HOLD_TIME_S="${HOLD_TIME_S:-3.0}"
ACCEPTANCE_RADIUS_M="${ACCEPTANCE_RADIUS_M:-0.35}"
MAX_HORIZONTAL_VELOCITY_M_S="${MAX_HORIZONTAL_VELOCITY_M_S:-0.8}"
MAX_VERTICAL_VELOCITY_M_S="${MAX_VERTICAL_VELOCITY_M_S:-0.5}"
MAX_HEADING_RATE_DEG_S="${MAX_HEADING_RATE_DEG_S:-60.0}"
MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S:-180.0}"
AUTO_RTL_AFTER_FINISH="${AUTO_RTL_AFTER_FINISH:-false}"
USE_LIVOX="${USE_LIVOX:-false}"
LAUNCH_OBSTACLE_SIM="${LAUNCH_OBSTACLE_SIM:-false}"
OBSTACLE_SIM_MODE="${OBSTACLE_SIM_MODE:-safe}"
OBSTACLE_DISTANCE_TOPIC="${OBSTACLE_DISTANCE_TOPIC:-/perception/min_obstacle_distance}"
OBSTACLE_STOP_DISTANCE_M="${OBSTACLE_STOP_DISTANCE_M:-2.0}"
OBSTACLE_ABORT_DISTANCE_M="${OBSTACLE_ABORT_DISTANCE_M:-1.0}"
OBSTACLE_HOLD_TIMEOUT_S="${OBSTACLE_HOLD_TIMEOUT_S:-5.0}"
ENABLE_OBSTACLE_HOLD="${ENABLE_OBSTACLE_HOLD:-true}"
LAUNCH_GZ_SCAN_DISTANCE="${LAUNCH_GZ_SCAN_DISTANCE:-false}"
GZ_SCAN_TOPIC="${GZ_SCAN_TOPIC:-/world/room_obstacles/model/x500_lidar_front_0/link/lidar_sensor_link/sensor/lidar/scan}"
TARGET_POINT_TOPIC="${TARGET_POINT_TOPIC:-/autonomy/target_ned}"
ACCEPT_RUNTIME_TARGET="${ACCEPT_RUNTIME_TARGET:-true}"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found"
  exit 1
fi

if [[ ! -d "${PX4_DIR}" ]]; then
  echo "PX4 directory not found: ${PX4_DIR}"
  exit 1
fi

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "ROS 2 workspace not built: ${WS_DIR}/install/setup.bash missing"
  exit 1
fi

if [[ ! -x "${QGC_DIR}/AppRun" ]]; then
  echo "QGroundControl Daily not found: ${QGC_DIR}/AppRun"
  exit 1
fi

PX4_GZ_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
WS_WORLDS_DIR="${WS_DIR}/sim/worlds"

install_custom_worlds() {
  if [[ ! -d "${WS_WORLDS_DIR}" ]]; then
    return 0
  fi

  if [[ ! -d "${PX4_GZ_WORLDS_DIR}" ]]; then
    echo "PX4 Gazebo worlds directory not found: ${PX4_GZ_WORLDS_DIR}"
    return 0
  fi

  if compgen -G "${WS_WORLDS_DIR}/*.sdf" >/dev/null; then
    cp -u "${WS_WORLDS_DIR}"/*.sdf "${PX4_GZ_WORLDS_DIR}/"
  fi
}

resolve_system_geolocation() {
  if [[ "${USE_SYSTEM_GEOLOCATION}" != "true" ]]; then
    return 0
  fi

  if [[ -n "${PX4_HOME_LAT}" && -n "${PX4_HOME_LON}" ]]; then
    return 0
  fi

  if ! command -v curl >/dev/null 2>&1; then
    echo "curl not found, skipping system geolocation lookup"
    return 0
  fi

  local geo_json lat lon
  geo_json="$(curl -s --max-time 5 https://ipapi.co/json/ || true)"
  if [[ -z "${geo_json}" ]]; then
    echo "System geolocation lookup failed, continuing without PX4_HOME_* override"
    return 0
  fi

  lat="$(printf '%s' "${geo_json}" | sed -n 's/.*"latitude":[[:space:]]*\([-0-9.]*\).*/\1/p' | head -1)"
  lon="$(printf '%s' "${geo_json}" | sed -n 's/.*"longitude":[[:space:]]*\([-0-9.]*\).*/\1/p' | head -1)"

  if [[ -n "${lat}" && -n "${lon}" ]]; then
    PX4_HOME_LAT="${PX4_HOME_LAT:-${lat}}"
    PX4_HOME_LON="${PX4_HOME_LON:-${lon}}"
    PX4_HOME_ALT="${PX4_HOME_ALT:-20}"
    echo "Using system geolocation estimate: lat=${PX4_HOME_LAT}, lon=${PX4_HOME_LON}, alt=${PX4_HOME_ALT}"
  else
    echo "System geolocation lookup returned no coordinates, continuing without PX4_HOME_* override"
  fi
}

resolve_system_geolocation
install_custom_worlds

mkdir -p "${WS_DIR}/.runtime"
mkdir -p "${WS_DIR}/.runtime/logs"

cat > "${WS_DIR}/.runtime/ros2_autonomy_env.sh" <<EOF
set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u
export LD_LIBRARY_PATH="${WS_DIR}/local/livox_sdk2/lib:\${LD_LIBRARY_PATH:-}"
EOF

cat > "${WS_DIR}/.runtime/start_ros2_autonomy.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source "${WS_DIR}/.runtime/ros2_autonomy_env.sh"
exec ros2 launch px4_autonomy_bringup autonomy_stack.launch.py \\
  mission_size_m:=${MISSION_SIZE_M} \\
  mission_altitude_m:=${MISSION_ALTITUDE_M} \\
  hold_time_s:=${HOLD_TIME_S} \\
  acceptance_radius_m:=${ACCEPTANCE_RADIUS_M} \\
  max_horizontal_velocity_m_s:=${MAX_HORIZONTAL_VELOCITY_M_S} \\
  max_vertical_velocity_m_s:=${MAX_VERTICAL_VELOCITY_M_S} \\
  max_heading_rate_deg_s:=${MAX_HEADING_RATE_DEG_S} \\
  mission_timeout_s:=${MISSION_TIMEOUT_S} \\
  auto_rtl_after_finish:=${AUTO_RTL_AFTER_FINISH} \\
  use_livox:=${USE_LIVOX} \\
  launch_obstacle_sim:=${LAUNCH_OBSTACLE_SIM} \\
  obstacle_sim_mode:=${OBSTACLE_SIM_MODE} \\
  obstacle_distance_topic:=${OBSTACLE_DISTANCE_TOPIC} \\
  obstacle_stop_distance_m:=${OBSTACLE_STOP_DISTANCE_M} \\
  obstacle_abort_distance_m:=${OBSTACLE_ABORT_DISTANCE_M} \\
  obstacle_hold_timeout_s:=${OBSTACLE_HOLD_TIMEOUT_S} \\
  enable_obstacle_hold:=${ENABLE_OBSTACLE_HOLD} \\
  target_point_topic:=${TARGET_POINT_TOPIC} \\
  accept_runtime_target:=${ACCEPT_RUNTIME_TARGET}
EOF
chmod +x "${WS_DIR}/.runtime/start_ros2_autonomy.sh"

cat > "${WS_DIR}/.runtime/start_gz_scan_distance.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source "${WS_DIR}/.runtime/ros2_autonomy_env.sh"
exec ros2 run px4_obstacle_tools gz_scan_min_distance --ros-args \\
  -p gz_scan_topic:=${GZ_SCAN_TOPIC} \\
  -p distance_topic:=${OBSTACLE_DISTANCE_TOPIC}
EOF
chmod +x "${WS_DIR}/.runtime/start_gz_scan_distance.sh"

cat > "${WS_DIR}/.runtime/wait_for_fmu.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source "${WS_DIR}/.runtime/ros2_autonomy_env.sh"
echo "Waiting for /fmu topics..."
until ros2 topic list 2>/dev/null | grep -q '^/fmu/'; do
  sleep 1
done
echo "/fmu topics detected."
sleep 1
exec "${WS_DIR}/.runtime/start_ros2_autonomy.sh"
EOF
chmod +x "${WS_DIR}/.runtime/wait_for_fmu.sh"

cat > "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd "${PX4_DIR}"
export PX4_HOME_LAT="${PX4_HOME_LAT}"
export PX4_HOME_LON="${PX4_HOME_LON}"
export PX4_HOME_ALT="${PX4_HOME_ALT}"
export PX4_HOME_YAW="${PX4_HOME_YAW}"
export GZ_SIM_RESOURCE_PATH="${PX4_GZ_WORLDS_DIR}:${WS_DIR}/sim/worlds:\${GZ_SIM_RESOURCE_PATH:-}"
if [[ "${PX4_WORLD}" == "default" ]]; then
  exec make px4_sitl ${PX4_MODEL}
else
  export PX4_GZ_WORLD="${PX4_WORLD}"
  exec make px4_sitl ${PX4_MODEL}
fi
EOF
chmod +x "${WS_DIR}/.runtime/start_px4_sitl.sh"

cat > "${WS_DIR}/.runtime/start_agent.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
exec MicroXRCEAgent udp4 -p ${AGENT_PORT}
EOF
chmod +x "${WS_DIR}/.runtime/start_agent.sh"

cat > "${WS_DIR}/.runtime/start_qgc.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
if pgrep -f "QGroundControl|${QGC_DIR}/AppRun" >/dev/null 2>&1; then
  echo "QGroundControl is already running; not starting a second instance."
  exit 0
fi
cd "${QGC_DIR}"
exec env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY ./AppRun
EOF
chmod +x "${WS_DIR}/.runtime/start_qgc.sh"

cat > "${WS_DIR}/.runtime/wait_for_qgc_start.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
echo "Waiting a few seconds for PX4 MAVLink to come up before starting QGC..."
sleep 8
exec "${WS_DIR}/.runtime/start_qgc.sh"
EOF
chmod +x "${WS_DIR}/.runtime/wait_for_qgc_start.sh"

cat > "${WS_DIR}/.runtime/launch_stack_terminal.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

LOG_DIR="${WS_DIR}/.runtime/logs"
rm -f "\${LOG_DIR}/px4.log" "\${LOG_DIR}/agent.log" "\${LOG_DIR}/ros2.log" "\${LOG_DIR}/qgc.log"
touch "\${LOG_DIR}/px4.log" "\${LOG_DIR}/agent.log" "\${LOG_DIR}/ros2.log" "\${LOG_DIR}/qgc.log"

cleanup() {
  pkill -P $$ || true
}
trap cleanup EXIT INT TERM

echo "[stack] starting PX4 SITL..."
"${WS_DIR}/.runtime/start_px4_sitl.sh" >"\${LOG_DIR}/px4.log" 2>&1 &
PX4_PID=\$!

sleep 2
echo "[stack] starting MicroXRCEAgent..."
"${WS_DIR}/.runtime/start_agent.sh" >"\${LOG_DIR}/agent.log" 2>&1 &
AGENT_PID=\$!

sleep 2
echo "[stack] starting QGroundControl..."
"${WS_DIR}/.runtime/wait_for_qgc_start.sh" >"\${LOG_DIR}/qgc.log" 2>&1 &
QGC_PID=\$!

sleep 2
echo "[stack] waiting for /fmu and starting ROS2 autonomy..."
"${WS_DIR}/.runtime/wait_for_fmu.sh" >"\${LOG_DIR}/ros2.log" 2>&1 &
ROS2_PID=\$!

GZ_SCAN_PID=""
if [[ "${LAUNCH_GZ_SCAN_DISTANCE}" == "true" ]]; then
  echo "[stack] starting Gazebo scan distance bridge..."
  "${WS_DIR}/.runtime/start_gz_scan_distance.sh" >"\${LOG_DIR}/gz_scan_distance.log" 2>&1 &
  GZ_SCAN_PID=\$!
fi

echo "[stack] pids: PX4=\${PX4_PID} Agent=\${AGENT_PID} ROS2=\${ROS2_PID} QGC=\${QGC_PID} GzScan=\${GZ_SCAN_PID:-disabled}"
echo "[stack] logs: \${LOG_DIR}/px4.log, \${LOG_DIR}/agent.log, \${LOG_DIR}/ros2.log, \${LOG_DIR}/qgc.log, \${LOG_DIR}/gz_scan_distance.log"
echo
echo "==== PX4 SITL ===="
tail -n 0 -F "\${LOG_DIR}/px4.log" &
TAIL_PX4=\$!
echo "==== MicroXRCEAgent ===="
tail -n 0 -F "\${LOG_DIR}/agent.log" &
TAIL_AGENT=\$!
echo "==== ROS2 Autonomy ===="
tail -n 0 -F "\${LOG_DIR}/ros2.log" &
TAIL_ROS2=\$!
echo "==== QGroundControl launcher ===="
tail -n 0 -F "\${LOG_DIR}/qgc.log" &
TAIL_QGC=\$!
if [[ "${LAUNCH_GZ_SCAN_DISTANCE}" == "true" ]]; then
  echo "==== Gazebo Scan Distance ===="
  tail -n 0 -F "\${LOG_DIR}/gz_scan_distance.log" &
  TAIL_GZ_SCAN=\$!
fi

wait \${PX4_PID} \${AGENT_PID} \${ROS2_PID} \${QGC_PID}
EOF
chmod +x "${WS_DIR}/.runtime/launch_stack_terminal.sh"

gnome-terminal --title="PX4 SITL" -- bash -lc "\"${WS_DIR}/.runtime/start_px4_sitl.sh\" | tee \"${WS_DIR}/.runtime/logs/px4.log\"; exec bash"
sleep 2
gnome-terminal --title="MicroXRCEAgent" -- bash -lc "\"${WS_DIR}/.runtime/start_agent.sh\" | tee \"${WS_DIR}/.runtime/logs/agent.log\"; exec bash"
sleep 2
gnome-terminal --title="QGroundControl" -- bash -lc "\"${WS_DIR}/.runtime/wait_for_qgc_start.sh\" | tee \"${WS_DIR}/.runtime/logs/qgc.log\"; exec bash"
sleep 2
gnome-terminal --title="ROS2 Autonomy" -- bash -lc "\"${WS_DIR}/.runtime/wait_for_fmu.sh\" | tee \"${WS_DIR}/.runtime/logs/ros2.log\"; exec bash"
if [[ "${LAUNCH_GZ_SCAN_DISTANCE}" == "true" ]]; then
  sleep 2
  gnome-terminal --title="Gazebo Scan Distance" -- bash -lc "\"${WS_DIR}/.runtime/start_gz_scan_distance.sh\" | tee \"${WS_DIR}/.runtime/logs/gz_scan_distance.log\"; exec bash"
fi

cat <<MSG
PX4 simulation stack started.

Terminal windows launched:
  - PX4 SITL
  - MicroXRCEAgent
  - ROS2 Autonomy
$(if [[ "${LAUNCH_GZ_SCAN_DISTANCE}" == "true" ]]; then echo "  - Gazebo Scan Distance"; fi)
  - QGroundControl

Logs are also written to:
  ${WS_DIR}/.runtime/logs/px4.log
  ${WS_DIR}/.runtime/logs/agent.log
  ${WS_DIR}/.runtime/logs/ros2.log
  ${WS_DIR}/.runtime/logs/gz_scan_distance.log
  ${WS_DIR}/.runtime/logs/qgc.log

GUI windows:
  - Gazebo Sim
  - QGroundControl Daily

Map and location behavior:
  - QGroundControl is started without proxy variables to improve online map loading.
  - By default, the script does not override PX4 home from network location.
  - Set USE_SYSTEM_GEOLOCATION=true if you want to try approximate IP-based home coordinates.
  - This is approximate network location, not GPS-grade position.
  - You can override it manually with PX4_HOME_LAT / PX4_HOME_LON / PX4_HOME_ALT.
  Example:
    PX4_HOME_LAT=30.2741 PX4_HOME_LON=120.1551 PX4_HOME_ALT=20 \\
    ${WS_DIR}/scripts/start_px4_sim.sh
  Indoor room world example:
    PX4_WORLD=room_obstacles ${WS_DIR}/scripts/start_px4_sim.sh
  Obstacle hold simulation example:
    LAUNCH_OBSTACLE_SIM=true OBSTACLE_SIM_MODE=hold ${WS_DIR}/scripts/start_px4_sim.sh
  Gazebo lidar obstacle hold example:
    PX4_MODEL=gz_x500_lidar_front LAUNCH_GZ_SCAN_DISTANCE=true \\
    OBSTACLE_STOP_DISTANCE_M=5.0 OBSTACLE_HOLD_TIMEOUT_S=0.0 \\
    ${WS_DIR}/scripts/start_px4_sim.sh

Suggested next steps:
  - Wait for Gazebo and QGC to connect.
  - Arm in Position mode.
  - Take off and stabilize.
  - Switch to ROS2 Autonomy.
  - Switch back to Position / RTL / Land to validate takeover.
MSG
