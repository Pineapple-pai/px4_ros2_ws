#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-/home/p/PX4-Autopilot}"
WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
QGC_DIR="${QGC_DIR:-/home/p/下载/PX4/qgc-daily-root/squashfs-root}"
AGENT_PORT="${AGENT_PORT:-8888}"
TERMINAL_LAYOUT="${TERMINAL_LAYOUT:-windows}"
KEEP_TERMINALS_OPEN="${KEEP_TERMINALS_OPEN:-true}"

# ============ 仿真配置 ============
# 仿真后端: gazebo-classic 或 gz（新版 Gazebo Ignition）
SIM_BACKEND="${SIM_BACKEND:-gazebo-classic}"
# PX4 机型模型
PX4_MODEL="${PX4_MODEL:-iris_rplidar}"
# Gazebo 世界。默认使用本仓库的房间环境，方便雷达测距一启动就看到障碍物。
PX4_WORLD="${PX4_WORLD:-room_obstacles}"

# PX4 家位置（可选，留空则使用 PX4 默认）
PX4_HOME_LAT="${PX4_HOME_LAT:-}"
PX4_HOME_LON="${PX4_HOME_LON:-}"
PX4_HOME_ALT="${PX4_HOME_ALT:-}"
PX4_HOME_YAW="${PX4_HOME_YAW:-}"
USE_SYSTEM_GEOLOCATION="${USE_SYSTEM_GEOLOCATION:-false}"

# ============ 自主模式配置 ============
MISSION_SIZE_M="${MISSION_SIZE_M:-2.5}"
MISSION_ALTITUDE_M="${MISSION_ALTITUDE_M:-2.0}"
HOLD_TIME_S="${HOLD_TIME_S:-3.0}"
ACCEPTANCE_RADIUS_M="${ACCEPTANCE_RADIUS_M:-0.35}"
MAX_HORIZONTAL_VELOCITY_M_S="${MAX_HORIZONTAL_VELOCITY_M_S:-0.8}"
MAX_VERTICAL_VELOCITY_M_S="${MAX_VERTICAL_VELOCITY_M_S:-0.5}"
MAX_HEADING_RATE_DEG_S="${MAX_HEADING_RATE_DEG_S:-60.0}"
MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S:-180.0}"
AUTO_RTL_AFTER_FINISH="${AUTO_RTL_AFTER_FINISH:-false}"
MISSION_WAYPOINTS_NED="${MISSION_WAYPOINTS_NED:-}"
MISSION_FILE="${MISSION_FILE:-}"

# ============ 避障配置 ============
ENABLE_OBSTACLE_AVOIDANCE="${ENABLE_OBSTACLE_AVOIDANCE:-true}"
OBSTACLE_DISTANCE_TOPIC="${OBSTACLE_DISTANCE_TOPIC:-/perception/min_obstacle_distance}"
OBSTACLE_STOP_DISTANCE_M="${OBSTACLE_STOP_DISTANCE_M:-2.0}"
OBSTACLE_ABORT_DISTANCE_M="${OBSTACLE_ABORT_DISTANCE_M:-1.0}"
OBSTACLE_HOLD_TIMEOUT_S="${OBSTACLE_HOLD_TIMEOUT_S:-5.0}"
ENABLE_LOCAL_AVOIDANCE="${ENABLE_LOCAL_AVOIDANCE:-true}"
AVOIDANCE_TRIGGER_DISTANCE_M="${AVOIDANCE_TRIGGER_DISTANCE_M:-3.0}"
AVOIDANCE_CLEARANCE_M="${AVOIDANCE_CLEARANCE_M:-2.0}"
AVOIDANCE_LATERAL_OFFSET_M="${AVOIDANCE_LATERAL_OFFSET_M:-1.5}"
AVOIDANCE_FORWARD_OFFSET_M="${AVOIDANCE_FORWARD_OFFSET_M:-2.0}"
AVOIDANCE_SPEED_M_S="${AVOIDANCE_SPEED_M_S:-0.45}"
OBSTACLE_DATA_TIMEOUT_S="${OBSTACLE_DATA_TIMEOUT_S:-1.0}"

# ============ 激光雷达配置 ============
# Gazebo-Classic 模式下，传感器话题取决于模型使用的 gazebo_ros plugin
# iris_rplidar 默认发布 LaserScan 到 /scan
LASER_SCAN_TOPIC="${LASER_SCAN_TOPIC:-/scan}"
TARGET_POINT_TOPIC="${TARGET_POINT_TOPIC:-/autonomy/target_ned}"
WAYPOINTS_TOPIC="${WAYPOINTS_TOPIC:-/autonomy/waypoints_ned}"
ACCEPT_RUNTIME_TARGET="${ACCEPT_RUNTIME_TARGET:-true}"

# ============ 功能开关 ============
USE_SIM_TIME="${USE_SIM_TIME:-false}"
USE_FASTLIO="${USE_FASTLIO:-false}"
USE_LIVOX="${USE_LIVOX:-false}"
USE_NAV2="${USE_NAV2:-false}"
LIVOX_CONFIG="${LIVOX_CONFIG:-${WS_DIR}/src/livox_ros_driver2/config/MID360_config.json}"
FASTLIO_CONFIG_PATH="${FASTLIO_CONFIG_PATH:-}"
FASTLIO_CONFIG_FILE="${FASTLIO_CONFIG_FILE:-mid360.yaml}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}"
CONTROL_SOURCE="${CONTROL_SOURCE:-}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${WS_DIR}/src/px4_autonomy_bringup/config/nav2_params.yaml}"
NAV2_CMD_VEL_TOPIC="${NAV2_CMD_VEL_TOPIC:-/cmd_vel}"
NAV2_FIXED_ALTITUDE_M="${NAV2_FIXED_ALTITUDE_M:-${MISSION_ALTITUDE_M}}"
NAV2_CMD_TIMEOUT_S="${NAV2_CMD_TIMEOUT_S:-0.5}"
NAV2_LOOKAHEAD_TIME_S="${NAV2_LOOKAHEAD_TIME_S:-1.0}"
NAV2_MIN_LOOKAHEAD_M="${NAV2_MIN_LOOKAHEAD_M:-0.35}"
NAV2_MAX_CMD_SPEED_M_S="${NAV2_MAX_CMD_SPEED_M_S:-0.45}"
NAV2_TURN_IN_PLACE_YAW_RATE_RAD_S="${NAV2_TURN_IN_PLACE_YAW_RATE_RAD_S:-0.35}"
NAV2_TURN_SPEED_SCALE="${NAV2_TURN_SPEED_SCALE:-0.25}"
NAV2_TURN_MAX_LOOKAHEAD_M="${NAV2_TURN_MAX_LOOKAHEAD_M:-0.20}"
NAV2_OBSTACLE_SLOWDOWN_DISTANCE_M="${NAV2_OBSTACLE_SLOWDOWN_DISTANCE_M:-3.0}"
NAV2_OBSTACLE_MIN_SPEED_M_S="${NAV2_OBSTACLE_MIN_SPEED_M_S:-0.10}"
NAV2_EMERGENCY_RETREAT_DISTANCE_M="${NAV2_EMERGENCY_RETREAT_DISTANCE_M:-0.8}"
NAV2_EMERGENCY_RETREAT_SPEED_M_S="${NAV2_EMERGENCY_RETREAT_SPEED_M_S:-0.25}"
NAV2_REQUIRE_ROS2_CONTROL="${NAV2_REQUIRE_ROS2_CONTROL:-true}"
NAV2_REQUIRED_NAV_STATE="${NAV2_REQUIRED_NAV_STATE:-23}"
NAV2_STATUS_TIMEOUT_S="${NAV2_STATUS_TIMEOUT_S:-1.0}"
QGC_AUTO_REQUEST_ROS2_MODE="${QGC_AUTO_REQUEST_ROS2_MODE:-true}"
QGC_MODE_REQUEST_PERIOD_S="${QGC_MODE_REQUEST_PERIOD_S:-0.25}"
QGC_MODE_REQUEST_HOLD_S="${QGC_MODE_REQUEST_HOLD_S:-30.0}"
QGC_REPOSITION_GOAL_BRIDGE="${QGC_REPOSITION_GOAL_BRIDGE:-true}"
NAV2_ODOM_SOURCE="${NAV2_ODOM_SOURCE:-px4_local}"
NAV2_CLOUD_TOPIC="${NAV2_CLOUD_TOPIC:-/autonomy/cloud_registered}"
LAUNCH_GZ_SCAN_TO_POINTCLOUD="${LAUNCH_GZ_SCAN_TO_POINTCLOUD:-}"
OCTOMAP_RESOLUTION="${OCTOMAP_RESOLUTION:-0.10}"
OCTOMAP_MIN_Z="${OCTOMAP_MIN_Z:--0.5}"
OCTOMAP_MAX_Z="${OCTOMAP_MAX_Z:-2.0}"
LAUNCH_OBSTACLE_SIM="${LAUNCH_OBSTACLE_SIM:-false}"
OBSTACLE_SIM_MODE="${OBSTACLE_SIM_MODE:-safe}"

if [[ -z "${CONTROL_SOURCE}" ]]; then
  if [[ "${USE_NAV2}" == "true" ]]; then
    CONTROL_SOURCE="nav2_cmd_vel"
  else
    CONTROL_SOURCE="mission"
  fi
fi

if [[ -z "${POINTCLOUD_TOPIC:-}" ]]; then
  POINTCLOUD_TOPIC="/livox/lidar"
fi

# ---------- 预处理检查 ----------
case "${TERMINAL_LAYOUT}" in
  windows|tabs|tmux|headless) ;;
  *)
    echo "Unsupported TERMINAL_LAYOUT=${TERMINAL_LAYOUT}. Use windows, tabs, tmux, or headless."
    exit 1
    ;;
esac

if [[ "${TERMINAL_LAYOUT}" == "windows" || "${TERMINAL_LAYOUT}" == "tabs" ]]; then
  if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal not found. Use TERMINAL_LAYOUT=tmux or TERMINAL_LAYOUT=headless."
    exit 1
  fi
fi

if [[ "${TERMINAL_LAYOUT}" == "tmux" ]] && ! command -v tmux >/dev/null 2>&1; then
  echo "tmux not found. Use TERMINAL_LAYOUT=windows/tabs/headless or install tmux."
  exit 1
fi

if [[ "${USE_NAV2}" == "true" ]]; then
  case "${NAV2_ODOM_SOURCE}" in
    px4_local|fastlio) ;;
    *)
      echo "Unsupported NAV2_ODOM_SOURCE=${NAV2_ODOM_SOURCE}. Use px4_local or fastlio."
      exit 1
      ;;
  esac

  if [[ "${NAV2_ODOM_SOURCE}" == "fastlio" && "${USE_FASTLIO}" != "true" ]]; then
    echo "NAV2_ODOM_SOURCE=fastlio requires USE_FASTLIO=true."
    echo "For Gazebo simulation without FAST-LIO point clouds, use the default NAV2_ODOM_SOURCE=px4_local."
    exit 1
  fi

  missing_nav2_pkgs=()
  for pkg in nav2_bringup nav2_msgs octomap_server; do
    if ! bash -lc "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && ros2 pkg prefix ${pkg} >/dev/null 2>&1"; then
      missing_nav2_pkgs+=("${pkg}")
    fi
  done
  if ((${#missing_nav2_pkgs[@]} > 0)); then
    echo "Missing ROS 2 Nav2 dependencies: ${missing_nav2_pkgs[*]}"
    echo "Install them with:"
    echo "  sudo apt update && sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-octomap-server"
    exit 1
  fi
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

# ============ 辅助函数 ============

PX4_GZ_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
WS_WORLDS_DIR="${WS_DIR}/sim/worlds"
PX4_GAZEBO_CLASSIC_MODELS_DIR="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
PX4_GAZEBO_CLASSIC_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"

check_gazebo_classic_model_dependencies() {
  local root_model_name="$1"
  local models_dir="$2"

  if [[ -z "${root_model_name}" || ! -d "${models_dir}" ]]; then
    return 0
  fi

  local -a queue=("${root_model_name}")
  local -A visited=()

  while ((${#queue[@]} > 0)); do
    local model_name="${queue[0]}"
    queue=("${queue[@]:1}")

    if [[ -n "${visited[${model_name}]:-}" ]]; then
      continue
    fi
    visited["${model_name}"]=1

    local model_dir="${models_dir}/${model_name}"
    local sdf_path=""

    if [[ ! -d "${model_dir}" ]]; then
      echo "Missing Gazebo Classic model dependency: ${model_name}"
      return 1
    fi

    if [[ -f "${model_dir}/model.config" ]]; then
      local sdf_rel
      sdf_rel="$(sed -n 's:.*<sdf[^>]*>\([^<]*\)</sdf>.*:\1:p' "${model_dir}/model.config" | head -1)"
      if [[ -n "${sdf_rel}" && -f "${model_dir}/${sdf_rel}" ]]; then
        sdf_path="${model_dir}/${sdf_rel}"
      fi
    fi

    if [[ -z "${sdf_path}" ]]; then
      sdf_path="$(find "${model_dir}" -maxdepth 1 -name '*.sdf' | head -1)"
    fi

    if [[ -z "${sdf_path}" || ! -f "${sdf_path}" ]]; then
      echo "Unable to resolve SDF for Gazebo Classic model: ${model_name}"
      return 1
    fi

    while IFS= read -r dependency_name; do
      [[ -z "${dependency_name}" ]] && continue
      if [[ -z "${visited[${dependency_name}]:-}" ]]; then
        queue+=("${dependency_name}")
      fi
    done < <(
      sed -n 's|.*<uri>model://\([^<[:space:]]*\)</uri>.*|\1|p' "${sdf_path}" \
        | sed 's|/.*||' \
        | sort -u
    )
  done
}

install_custom_worlds() {
  if [[ ! -d "${WS_WORLDS_DIR}" ]]; then
    return 0
  fi

  if [[ -d "${PX4_GZ_WORLDS_DIR}" ]]; then
    if compgen -G "${WS_WORLDS_DIR}/*.sdf" >/dev/null; then
      cp -u "${WS_WORLDS_DIR}"/*.sdf "${PX4_GZ_WORLDS_DIR}/"
    fi
  else
    echo "PX4 Gazebo worlds directory not found: ${PX4_GZ_WORLDS_DIR}"
  fi

  if [[ -d "${PX4_GAZEBO_CLASSIC_WORLDS_DIR}" ]]; then
    local world_file
    for world_file in "${WS_WORLDS_DIR}"/*.sdf; do
      [[ -f "${world_file}" ]] || continue
      cp -u "${world_file}" \
        "${PX4_GAZEBO_CLASSIC_WORLDS_DIR}/$(basename "${world_file}" .sdf).world"
    done
  else
    echo "PX4 Gazebo Classic worlds directory not found: ${PX4_GAZEBO_CLASSIC_WORLDS_DIR}"
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

# ============ 执行预处理 ============
resolve_system_geolocation
install_custom_worlds

if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  CLASSIC_MAKE_TARGET="gazebo-classic_${PX4_MODEL}"
  case "${PX4_WORLD}" in
    empty|none)
      CLASSIC_MAKE_TARGET="gazebo-classic_${PX4_MODEL}"
      ;;
    baylands|ksql_airport|mcmillan_airfield|ramped_up_wind|sonoma_raceway|warehouse|windy|yosemite)
      CLASSIC_MAKE_TARGET="gazebo-classic_${PX4_MODEL}__${PX4_WORLD}"
      ;;
    *)
      CLASSIC_MAKE_TARGET="gazebo-classic_${PX4_MODEL}"
      ;;
  esac

  if ! check_gazebo_classic_model_dependencies "${PX4_MODEL}" "${PX4_GAZEBO_CLASSIC_MODELS_DIR}"; then
    echo "Gazebo Classic model dependency check failed for PX4_MODEL=${PX4_MODEL}"
    echo "Please fix the missing model(s) under: ${PX4_GAZEBO_CLASSIC_MODELS_DIR}"
    exit 1
  fi
fi

mkdir -p "${WS_DIR}/.runtime"
mkdir -p "${WS_DIR}/.runtime/logs"

# ============ 生成环境脚本 ============
cat > "${WS_DIR}/.runtime/ros2_autonomy_env.sh" <<EOF
set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
export LD_LIBRARY_PATH="${WS_DIR}/local/livox_sdk2/lib:\${LD_LIBRARY_PATH:-}"
set -u
EOF

if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  GZ_SCAN_DISTANCE_ARG="launch_gz_scan_distance:=true"

  # iris_rplidar.sdf 中包含了两个激光雷达模型：
  #   1) lidar  (model://lidar)      → samples=1, 使用 PX4 原生 libgazebo_lidar_plugin.so
  #   2) rplidar (model://rplidar)   → samples=360, 已改用 libgazebo_lidar_plugin.so（不依赖 ROS）
  #
  # rplidar 的激光传感器 samples=360，提供 360° 全方位测距，
  # libgazebo_lidar_plugin.so 会将所有射线的距离中的最小值发布为 Range 消息
  # （含 current_distance 字段），比 samples=1 的 lidar 模型更适合避障。
  #
  # 话题路径：Gazebo Classic 同时暴露 Range 和 LaserScanStamped 话题。
  # 实测 rplidar 的 360° LaserScanStamped 话题稳定输出完整 ranges：
  #   /gazebo/<world>/<root_model>/rplidar/link/laser/scan
  _GZ_WORLD_TOPIC="${PX4_WORLD}"
  if [[ "${PX4_WORLD}" == "empty" || "${PX4_WORLD}" == "none" ]]; then
    _GZ_WORLD_TOPIC="default"
  fi
  _DEFAULT_TOPIC="/gazebo/${_GZ_WORLD_TOPIC}/${PX4_MODEL}/rplidar/link/laser/scan"
  GZ_SCAN_TOPIC_ARG="gz_scan_topic:=${GZ_SCAN_TOPIC:-${_DEFAULT_TOPIC}}"

  # gazebo-classic 模式下 start_px4_sitl.sh 会 unset ROS_VERSION，
  # 导致 libgazebo_ros_laser.so 无法发布 ROS2 话题 /scan，
  # 因此只能通过 gz_scan_min_distance 读取 Gazebo 原生传输层主题。
  ENABLE_LASERSCAN_MIN_DISTANCE="false"
  if [[ -z "${LAUNCH_GZ_SCAN_TO_POINTCLOUD}" ]]; then
    LAUNCH_GZ_SCAN_TO_POINTCLOUD="${USE_NAV2}"
  fi
else
  GZ_SCAN_DISTANCE_ARG="launch_gz_scan_distance:=false"
  GZ_SCAN_TOPIC_ARG=""
  if [[ -z "${LAUNCH_GZ_SCAN_TO_POINTCLOUD}" ]]; then
    LAUNCH_GZ_SCAN_TO_POINTCLOUD="false"
  fi
fi

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
  use_sim_time:=${USE_SIM_TIME} \\
  use_fastlio:=${USE_FASTLIO} \\
  use_livox:=${USE_LIVOX} \\
  use_nav2:=${USE_NAV2} \\
  livox_config:=${LIVOX_CONFIG} \\
  fastlio_config_file:=${FASTLIO_CONFIG_FILE} \\
  fastlio_rviz:=${FASTLIO_RVIZ} \\
  control_source:=${CONTROL_SOURCE} \\
  nav2_params_file:=${NAV2_PARAMS_FILE} \\
  nav2_cmd_vel_topic:=${NAV2_CMD_VEL_TOPIC} \\
  nav2_fixed_altitude_m:=${NAV2_FIXED_ALTITUDE_M} \\
  nav2_cmd_timeout_s:=${NAV2_CMD_TIMEOUT_S} \\
  nav2_lookahead_time_s:=${NAV2_LOOKAHEAD_TIME_S} \\
  nav2_min_lookahead_m:=${NAV2_MIN_LOOKAHEAD_M} \\
  nav2_max_cmd_speed_m_s:=${NAV2_MAX_CMD_SPEED_M_S} \\
  nav2_turn_in_place_yaw_rate_rad_s:=${NAV2_TURN_IN_PLACE_YAW_RATE_RAD_S} \\
  nav2_turn_speed_scale:=${NAV2_TURN_SPEED_SCALE} \\
  nav2_turn_max_lookahead_m:=${NAV2_TURN_MAX_LOOKAHEAD_M} \\
  nav2_obstacle_slowdown_distance_m:=${NAV2_OBSTACLE_SLOWDOWN_DISTANCE_M} \\
  nav2_obstacle_min_speed_m_s:=${NAV2_OBSTACLE_MIN_SPEED_M_S} \\
  nav2_emergency_retreat_distance_m:=${NAV2_EMERGENCY_RETREAT_DISTANCE_M} \\
  nav2_emergency_retreat_speed_m_s:=${NAV2_EMERGENCY_RETREAT_SPEED_M_S} \\
  nav2_require_ros2_control:=${NAV2_REQUIRE_ROS2_CONTROL} \\
  nav2_required_nav_state:=${NAV2_REQUIRED_NAV_STATE} \\
  nav2_status_timeout_s:=${NAV2_STATUS_TIMEOUT_S} \\
  qgc_auto_request_ros2_mode:=${QGC_AUTO_REQUEST_ROS2_MODE} \\
  qgc_mode_request_period_s:=${QGC_MODE_REQUEST_PERIOD_S} \\
  qgc_mode_request_hold_s:=${QGC_MODE_REQUEST_HOLD_S} \\
  qgc_reposition_goal_bridge:=${QGC_REPOSITION_GOAL_BRIDGE} \\
  nav2_odom_source:=${NAV2_ODOM_SOURCE} \\
  nav2_cloud_topic:=${NAV2_CLOUD_TOPIC} \\
  launch_gz_scan_to_pointcloud:=${LAUNCH_GZ_SCAN_TO_POINTCLOUD} \\
  octomap_resolution:=${OCTOMAP_RESOLUTION} \\
  octomap_min_z:=${OCTOMAP_MIN_Z} \\
  octomap_max_z:=${OCTOMAP_MAX_Z} \\
  launch_obstacle_sim:=${LAUNCH_OBSTACLE_SIM} \\
  obstacle_sim_mode:=${OBSTACLE_SIM_MODE} \\
  obstacle_distance_topic:=${OBSTACLE_DISTANCE_TOPIC} \\
  obstacle_stop_distance_m:=${OBSTACLE_STOP_DISTANCE_M} \\
  obstacle_abort_distance_m:=${OBSTACLE_ABORT_DISTANCE_M} \\
  obstacle_hold_timeout_s:=${OBSTACLE_HOLD_TIMEOUT_S} \\
  enable_obstacle_hold:=${ENABLE_OBSTACLE_AVOIDANCE} \\
  enable_local_avoidance:=${ENABLE_LOCAL_AVOIDANCE} \\
  avoidance_trigger_distance_m:=${AVOIDANCE_TRIGGER_DISTANCE_M} \\
  avoidance_clearance_m:=${AVOIDANCE_CLEARANCE_M} \\
  avoidance_lateral_offset_m:=${AVOIDANCE_LATERAL_OFFSET_M} \\
  avoidance_forward_offset_m:=${AVOIDANCE_FORWARD_OFFSET_M} \\
  avoidance_speed_m_s:=${AVOIDANCE_SPEED_M_S} \\
  obstacle_data_timeout_s:=${OBSTACLE_DATA_TIMEOUT_S} \\
  target_point_topic:=${TARGET_POINT_TOPIC} \\
  waypoints_topic:=${WAYPOINTS_TOPIC} \\
  accept_runtime_target:=${ACCEPT_RUNTIME_TARGET} \\
  ${GZ_SCAN_DISTANCE_ARG} \\
  launch_gz_six_direction_distance:=true \\
EOF

if [[ -n "${GZ_SCAN_TOPIC_ARG}" ]]; then
  cat >> "${WS_DIR}/.runtime/start_ros2_autonomy.sh" <<EOF
  ${GZ_SCAN_TOPIC_ARG} \\
EOF
fi

if [[ -n "${MISSION_WAYPOINTS_NED}" ]]; then
  cat >> "${WS_DIR}/.runtime/start_ros2_autonomy.sh" <<EOF
  mission_waypoints_ned:="${MISSION_WAYPOINTS_NED}" \\
EOF
fi

if [[ -n "${MISSION_FILE}" ]]; then
  cat >> "${WS_DIR}/.runtime/start_ros2_autonomy.sh" <<EOF
  mission_file:="${MISSION_FILE}" \\
EOF
fi

if [[ -n "${FASTLIO_CONFIG_PATH}" ]]; then
  cat >> "${WS_DIR}/.runtime/start_ros2_autonomy.sh" <<EOF
  fastlio_config_path:=${FASTLIO_CONFIG_PATH} \\
EOF
fi

cat >> "${WS_DIR}/.runtime/start_ros2_autonomy.sh" <<EOF
  pointcloud_topic:=${POINTCLOUD_TOPIC} \\
  launch_laserscan_min_distance:=false \\
  launch_pointcloud_min_distance:=false \\
  launch_pointcloud_front_min_distance:=false \\
  launch_pointcloud_up_min_distance:=false \\
  launch_pointcloud_down_min_distance:=false
EOF
chmod +x "${WS_DIR}/.runtime/start_ros2_autonomy.sh"

# 等待 FMU 出现后再启动 ROS2
cat > "${WS_DIR}/.runtime/wait_for_fmu.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source "${WS_DIR}/.runtime/ros2_autonomy_env.sh"
echo "Waiting for /fmu topics..."
deadline=\$((SECONDS + 90))
until timeout 5 ros2 topic list 2>/dev/null | grep -q '^/fmu/'; do
  if ((SECONDS >= deadline)); then
    echo "WARNING: /fmu topics were not discovered within 90s; starting ROS2 autonomy anyway."
    echo "         px4_autonomy_mode and bridges will keep waiting for PX4 data."
    break
  fi
  sleep 1
done
if timeout 5 ros2 topic list 2>/dev/null | grep -q '^/fmu/'; then
  echo "/fmu topics detected."
fi
sleep 1
exec "${WS_DIR}/.runtime/start_ros2_autonomy.sh"
EOF
chmod +x "${WS_DIR}/.runtime/wait_for_fmu.sh"

# ============ 生成 PX4 SITL 启动脚本 ============
cat > "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "${PX4_DIR}"

# 设置 PX4 SIM 模型和环境变量
export PX4_SIM_MODEL="${PX4_MODEL}"
export PX4_GZ_MODEL="${PX4_MODEL}"
export PX4_SIMULATOR="${SIM_BACKEND}"

# 家位置（可选）
EOF

# 添加家位置设置
if [[ -n "${PX4_HOME_LAT}" && -n "${PX4_HOME_LON}" && -n "${PX4_HOME_ALT}" ]]; then
  cat >> "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF
export PX4_HOME_LAT="${PX4_HOME_LAT}"
export PX4_HOME_LON="${PX4_HOME_LON}"
export PX4_HOME_ALT="${PX4_HOME_ALT}"
export PX4_HOME_YAW="${PX4_HOME_YAW}"
EOF
fi

# Gazebo-Classic 需要额外设置环境变量
if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  cat >> "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF

# 先 source ROS2 环境，这样 gazebo_ros 插件才能正常初始化
# ROS2 setup.bash 中可能引用未定义变量（如 AMENT_TRACE_SETUP_FILES），
# 需要用 set +u 避免 set -euo pipefail 报错
set +u
source /opt/ros/humble/setup.bash
set -u

# Gazebo-Classic 环境设置
# 先给变量赋默认空值，防止 setup_gazebo.bash 引用未绑定变量时报错退出
export GAZEBO_PLUGIN_PATH="\${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="\${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="\${LD_LIBRARY_PATH:-}"
source "${PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash" "${PX4_DIR}" "${PX4_DIR}/build/px4_sitl_default"

if [[ "${PX4_WORLD}" != "empty" && "${PX4_WORLD}" != "none" ]]; then
  export PX4_SIM_WORLD="${PX4_WORLD}"
  export PX4_SITL_WORLD="${PX4_GAZEBO_CLASSIC_WORLDS_DIR}/${PX4_WORLD}.world"
fi

# 恢复 DISPLAY 和 XAUTHORITY 环境变量
# gnome-terminal -- bash -lc ... 创建的 login shell 不会继承父环境的 DISPLAY 和 XAUTHORITY，
# 导致 gzclient 找不到 X11 显示，Gazebo 图形界面无法出现。
export DISPLAY="\${DISPLAY:-${DISPLAY:-:0}}"
export XAUTHORITY="\${XAUTHORITY:-${XAUTHORITY:-}}"

# 检查 DISPLAY 是否可用，如果不可用则强制 HEADLESS 模式
if ! timeout 1 xdpyinfo -display "\$DISPLAY" >/dev/null 2>&1; then
  echo "WARNING: DISPLAY=\$DISPLAY is not accessible. Gazebo GUI will not appear."
  echo "Set HEADLESS=1 or check your X11 configuration."
  export HEADLESS=1
fi

# source /opt/ros/humble/setup.bash 会设置 ROS_VERSION=2，
# 这会导致 sitl_run.sh 检测到 ROS2 并让 gzserver 加载全局 ROS2 插件
# (-s libgazebo_ros_init.so -s libgazebo_ros_factory.so)。
# 这些插件在 spawn 模型时需等待 ROS2 初始化，未启动完整 ROS2 环境时会阻塞。
# 因此需要 unset ROS_VERSION，让 gzserver 以纯 Gazebo 模式运行。
# rplidar 模型已改用 libgazebo_lidar_plugin.so（原生 Gazebo 插件）。
unset ROS_VERSION

# 不依赖 PX4 在后台隐式拉起 gzclient。
# 改为由本脚本在单独终端显式启动 GUI，避免 GUI 进程静默退出但终端不可见。
export HEADLESS=1
export PX4_NO_FOLLOW_MODE=1

exec make px4_sitl ${CLASSIC_MAKE_TARGET}
EOF
else
  # 新版 Gazebo (gz)
  cat >> "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF

# 新版 Gazebo (Ignition)
export GZ_SIM_SYSTEM_PLUGIN_PATH="\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:${PX4_DIR}/build/px4_sitl_default/build_gz/lib"

exec make px4_sitl gz_${PX4_MODEL}
EOF
fi

chmod +x "${WS_DIR}/.runtime/start_px4_sitl.sh"

if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  cat > "${WS_DIR}/.runtime/start_gazebo_gui.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "${PX4_DIR}"

set +u
source /opt/ros/humble/setup.bash
set -u

export GAZEBO_PLUGIN_PATH="\${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="\${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="\${LD_LIBRARY_PATH:-}"
source "${PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash" "${PX4_DIR}" "${PX4_DIR}/build/px4_sitl_default"

export DISPLAY="\${DISPLAY:-${DISPLAY:-:0}}"
export XAUTHORITY="\${XAUTHORITY:-${XAUTHORITY:-}}"

if ! command -v gzclient >/dev/null 2>&1; then
  echo "gzclient not found. Please install gazebo-classic GUI components."
  exit 1
fi

if ! timeout 1 xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
  echo "DISPLAY=$DISPLAY is not accessible, cannot launch Gazebo GUI."
  exit 1
fi

echo "Waiting for gzserver to become ready..."
until timeout 2 gz topic -l >/dev/null 2>&1; do
  sleep 1
done

echo "Starting Gazebo GUI (gzclient)..."
exec nice -n 10 gzclient --verbose
EOF
  chmod +x "${WS_DIR}/.runtime/start_gazebo_gui.sh"
fi

# ============ 生成 MicroXRCEAgent 启动脚本 ============
cat > "${WS_DIR}/.runtime/start_agent.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
exec MicroXRCEAgent udp4 -p ${AGENT_PORT}
EOF
chmod +x "${WS_DIR}/.runtime/start_agent.sh"

# ============ 生成 QGC 启动脚本 ============
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
sleep 10
exec "${WS_DIR}/.runtime/start_qgc.sh"
EOF
chmod +x "${WS_DIR}/.runtime/wait_for_qgc_start.sh"

# ============ 启动终端 ============

terminal_total=4
if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  terminal_total=5
fi

component_titles=("PX4 SITL + Gazebo")
component_scripts=("${WS_DIR}/.runtime/start_px4_sitl.sh")
component_logs=("${WS_DIR}/.runtime/logs/px4.log")

if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  component_titles+=("Gazebo GUI")
  component_scripts+=("${WS_DIR}/.runtime/start_gazebo_gui.sh")
  component_logs+=("${WS_DIR}/.runtime/logs/gzclient.log")
fi

component_titles+=("MicroXRCEAgent")
component_scripts+=("${WS_DIR}/.runtime/start_agent.sh")
component_logs+=("${WS_DIR}/.runtime/logs/agent.log")

component_titles+=("QGroundControl")
component_scripts+=("${WS_DIR}/.runtime/wait_for_qgc_start.sh")
component_logs+=("${WS_DIR}/.runtime/logs/qgc.log")

component_titles+=("ROS2 Autonomy")
component_scripts+=("${WS_DIR}/.runtime/wait_for_fmu.sh")
component_logs+=("${WS_DIR}/.runtime/logs/ros2.log")

terminal_command() {
  local script_path="$1"
  local log_path="$2"
  if [[ "${KEEP_TERMINALS_OPEN}" == "true" ]]; then
    printf '"%s" 2>&1 | tee "%s"; exec bash' "${script_path}" "${log_path}"
  else
    printf '"%s" 2>&1 | tee "%s"' "${script_path}" "${log_path}"
  fi
}

run_in_terminal() {
  local title="$1"
  local script_path="$2"
  local log_path="$3"
  gnome-terminal --title="${title}" -- bash -c "$(terminal_command "${script_path}" "${log_path}")"
}

append_terminal_tab() {
  local -n _args_ref="$1"
  local title="$2"
  local script_path="$3"
  local log_path="$4"
  _args_ref+=(--tab --title="${title}" -- bash -c "$(terminal_command "${script_path}" "${log_path}")")
}

run_in_tmux() {
  local session_name="px4-sim"
  if tmux has-session -t "${session_name}" 2>/dev/null; then
    tmux kill-session -t "${session_name}"
  fi

  tmux new-session -d -s "${session_name}" -n "PX4" \
    "bash -lc '\"${component_scripts[0]}\" 2>&1 | tee \"${component_logs[0]}\"'"

  local i
  for ((i = 1; i < ${#component_titles[@]}; ++i)); do
    tmux new-window -t "${session_name}" -n "${component_titles[$i]}" \
      "bash -lc '\"${component_scripts[$i]}\" 2>&1 | tee \"${component_logs[$i]}\"'"
    sleep 1
  done

  echo ">>> tmux session started: ${session_name}"
  echo ">>> Attach with: tmux attach -t ${session_name}"
}

run_headless() {
  local i
  if ! command -v script >/dev/null 2>&1; then
    echo "script(1) not found. Headless PX4 needs a pseudo-terminal; install util-linux or use TERMINAL_LAYOUT=windows/tabs."
    exit 1
  fi

  for ((i = 0; i < ${#component_titles[@]}; ++i)); do
    if [[ "${component_titles[$i]}" == "Gazebo GUI" || "${component_titles[$i]}" == "QGroundControl" ]]; then
      echo ">>> 跳过 GUI 组件（headless）: ${component_titles[$i]}"
      continue
    fi
    echo ">>> 后台启动: ${component_titles[$i]}"
    if [[ "${component_titles[$i]}" == "PX4 SITL + Gazebo" ]]; then
      nohup setsid script -q -f -c "\"${component_scripts[$i]}\"" "${component_logs[$i]}" >/dev/null 2>&1 &
    else
      nohup setsid bash -lc 'exec "$1"' _ "${component_scripts[$i]}" > "${component_logs[$i]}" 2>&1 &
    fi
    echo "$!" > "${component_logs[$i]}.pid"
    sleep 1
  done
}

echo ">>> 启动 ${terminal_total} 个组件，终端布局: ${TERMINAL_LAYOUT}"

if [[ "${TERMINAL_LAYOUT}" == "windows" ]]; then
  for ((i = 0; i < ${#component_titles[@]}; ++i)); do
    echo ">>> 启动窗口 $((i + 1))/${#component_titles[@]}: ${component_titles[$i]}"
    run_in_terminal "${component_titles[$i]}" "${component_scripts[$i]}" "${component_logs[$i]}"
    sleep 2
  done
elif [[ "${TERMINAL_LAYOUT}" == "tabs" ]]; then
  terminal_args=()
  for ((i = 0; i < ${#component_titles[@]}; ++i)); do
    append_terminal_tab terminal_args "${component_titles[$i]}" "${component_scripts[$i]}" "${component_logs[$i]}"
  done

  echo ">>> 启动单个 GNOME Terminal 窗口，并在其中创建 ${terminal_total} 个标签页"
  gnome-terminal "${terminal_args[@]}"
elif [[ "${TERMINAL_LAYOUT}" == "tmux" ]]; then
  run_in_tmux
else
  run_headless
fi

# ============ 输出提示 ============
cat <<MSG

================================================================
  PX4 仿真堆栈已启动！
================================================================

组件:
  标签页/终端 - PX4 SITL:        $(basename ${WS_DIR}/.runtime/logs/px4.log)
$(if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then echo "  标签页/终端 - Gazebo GUI:      $(basename ${WS_DIR}/.runtime/logs/gzclient.log)"; fi)
  标签页/终端 - MicroXRCEAgent:  $(basename ${WS_DIR}/.runtime/logs/agent.log)
  标签页/终端 - QGroundControl:  $(basename ${WS_DIR}/.runtime/logs/qgc.log)
  标签页/终端 - ROS2 Autonomy:   $(basename ${WS_DIR}/.runtime/logs/ros2.log)

仿真配置:
  - 后端:        ${SIM_BACKEND}
  - 模型:        ${PX4_MODEL}
  - 世界:        ${PX4_WORLD}
  - 避障:        ${ENABLE_OBSTACLE_AVOIDANCE}
  - 局部绕障:    ${ENABLE_LOCAL_AVOIDANCE}
  - Livox:       ${USE_LIVOX}
  - FAST-LIO2:   ${USE_FASTLIO}
  - Nav2:        ${USE_NAV2}
  - 控制源:      ${CONTROL_SOURCE}
  - Nav2 odom:   ${NAV2_ODOM_SOURCE}
  - Nav2 cmd:    ${NAV2_CMD_VEL_TOPIC}
  - Nav2 高度:   ${NAV2_FIXED_ALTITUDE_M} m
  - Nav2 map:    ${NAV2_CLOUD_TOPIC} -> /map
  - 点云输入:    ${POINTCLOUD_TOPIC}

终端布局:       ${TERMINAL_LAYOUT}（windows=多窗口，tabs=单窗口多标签，tmux/headless=无桌面验证）
Gazebo 界面:    $(if [[ "${TERMINAL_LAYOUT}" == "headless" ]]; then echo "headless 模式跳过 GUI，仅启动 gzserver"; else echo "自动启动（gzserver + 独立标签页/终端中的 gzclient）"; fi)
QGC 界面:       $(if [[ "${TERMINAL_LAYOUT}" == "headless" ]]; then echo "headless 模式跳过"; else echo "自动启动"; fi)

等待 Gazebo 加载完成后:
  1. 在 QGC 中连接（自动检测）
  2. 切换到 Position 模式 → Arm → Takeoff
  3. 切换到 Offboard 模式（ROS2 Autonomy 自动接管）
  4. 观察避障行为

使用示例（带激光雷达避障）:
  # 默认: iris_rplidar + room_obstacles 房间环境
  ./scripts/start_px4_sim.sh

  # 显式指定房间环境
  PX4_WORLD=room_obstacles ./scripts/start_px4_sim.sh

  # 新版 Gazebo (若安装了 gz-sim)
  SIM_BACKEND=gz PX4_MODEL=x500_lidar_front ./scripts/start_px4_sim.sh

  # 启用 FAST-LIO2（需要真实 Livox/MID360 数据或等价 /livox/lidar + /livox/imu 输入）
  USE_FASTLIO=true USE_LIVOX=true ./scripts/start_px4_sim.sh

  # 启用 Nav2 固定高度规划链路（需要 FAST-LIO2 点云/里程计和 Nav2/octomap 依赖）
  USE_FASTLIO=true USE_NAV2=true ./scripts/start_px4_sim.sh

  # 无桌面窗口验证
  TERMINAL_LAYOUT=headless ./scripts/start_px4_sim.sh

停止: 关闭终端窗口即可
================================================================
MSG
