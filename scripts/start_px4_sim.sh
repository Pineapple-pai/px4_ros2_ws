#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-/home/p/PX4-Autopilot}"
WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
QGC_DIR="${QGC_DIR:-/home/p/下载/PX4/qgc-daily-root/squashfs-root}"
AGENT_PORT="${AGENT_PORT:-8888}"
TERMINAL_LAYOUT="${TERMINAL_LAYOUT:-tabs}"

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
USE_SIM_TIME="${USE_SIM_TIME:-true}"
USE_FASTLIO="${USE_FASTLIO:-false}"
USE_LIVOX="${USE_LIVOX:-false}"
FASTLIO_CONFIG_PATH="${FASTLIO_CONFIG_PATH:-}"
FASTLIO_CONFIG_FILE="${FASTLIO_CONFIG_FILE:-mid360.yaml}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}"
LAUNCH_OBSTACLE_SIM="${LAUNCH_OBSTACLE_SIM:-false}"
OBSTACLE_SIM_MODE="${OBSTACLE_SIM_MODE:-safe}"

if [[ -z "${POINTCLOUD_TOPIC:-}" ]]; then
  if [[ "${USE_FASTLIO}" == "true" ]]; then
    POINTCLOUD_TOPIC="/autonomy/cloud_registered"
  else
    POINTCLOUD_TOPIC="/livox/lidar"
  fi
fi

# ---------- 预处理检查 ----------
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
else
  GZ_SCAN_DISTANCE_ARG="launch_gz_scan_distance:=false"
  GZ_SCAN_TOPIC_ARG=""
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
  mission_waypoints_ned:="${MISSION_WAYPOINTS_NED}" \\
  mission_file:="${MISSION_FILE}" \\
  use_sim_time:=${USE_SIM_TIME} \\
  use_fastlio:=${USE_FASTLIO} \\
  use_livox:=${USE_LIVOX} \\
  fastlio_config_file:=${FASTLIO_CONFIG_FILE} \\
  fastlio_rviz:=${FASTLIO_RVIZ} \\
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
until ros2 topic list 2>/dev/null | grep -q '^/fmu/'; do
  sleep 1
done
echo "/fmu topics detected."
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

run_in_terminal() {
  local title="$1"
  local script_path="$2"
  local log_path="$3"
  gnome-terminal --title="${title}" -- bash -c "\"${script_path}\" 2>&1 | tee \"${log_path}\"; exec bash"
}

append_terminal_tab() {
  local -n _args_ref="$1"
  local title="$2"
  local script_path="$3"
  local log_path="$4"
  _args_ref+=(--tab --title="${title}" -- bash -c "\"${script_path}\" 2>&1 | tee \"${log_path}\"; exec bash")
}

echo ">>> 启动 ${terminal_total} 个组件，终端布局: ${TERMINAL_LAYOUT}"

if [[ "${TERMINAL_LAYOUT}" == "windows" ]]; then
  echo ">>> 启动终端 1/${terminal_total}: PX4 SITL + Gazebo-Classic (${PX4_MODEL})"
  run_in_terminal "PX4 SITL + Gazebo" "${WS_DIR}/.runtime/start_px4_sitl.sh" "${WS_DIR}/.runtime/logs/px4.log"
  sleep 3

  if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
    echo ">>> 启动终端 2/${terminal_total}: Gazebo GUI"
    run_in_terminal "Gazebo GUI" "${WS_DIR}/.runtime/start_gazebo_gui.sh" "${WS_DIR}/.runtime/logs/gzclient.log"
    sleep 2
  fi

  agent_index=2
  qgc_index=3
  ros2_index=4
  if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
    agent_index=3
    qgc_index=4
    ros2_index=5
  fi

  echo ">>> 启动终端 ${agent_index}/${terminal_total}: MicroXRCEAgent (端口 ${AGENT_PORT})"
  run_in_terminal "MicroXRCEAgent" "${WS_DIR}/.runtime/start_agent.sh" "${WS_DIR}/.runtime/logs/agent.log"
  sleep 2

  echo ">>> 启动终端 ${qgc_index}/${terminal_total}: QGroundControl"
  run_in_terminal "QGroundControl" "${WS_DIR}/.runtime/wait_for_qgc_start.sh" "${WS_DIR}/.runtime/logs/qgc.log"
  sleep 2

  echo ">>> 启动终端 ${ros2_index}/${terminal_total}: ROS2 Autonomy（等待 /fmu 就绪后启动）"
  run_in_terminal "ROS2 Autonomy" "${WS_DIR}/.runtime/wait_for_fmu.sh" "${WS_DIR}/.runtime/logs/ros2.log"
else
  terminal_args=()
  append_terminal_tab terminal_args "PX4 SITL" "${WS_DIR}/.runtime/start_px4_sitl.sh" "${WS_DIR}/.runtime/logs/px4.log"

  if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
    append_terminal_tab terminal_args "Gazebo GUI" "${WS_DIR}/.runtime/start_gazebo_gui.sh" "${WS_DIR}/.runtime/logs/gzclient.log"
  fi

  append_terminal_tab terminal_args "MicroXRCEAgent" "${WS_DIR}/.runtime/start_agent.sh" "${WS_DIR}/.runtime/logs/agent.log"
  append_terminal_tab terminal_args "QGroundControl" "${WS_DIR}/.runtime/wait_for_qgc_start.sh" "${WS_DIR}/.runtime/logs/qgc.log"
  append_terminal_tab terminal_args "ROS2 Autonomy" "${WS_DIR}/.runtime/wait_for_fmu.sh" "${WS_DIR}/.runtime/logs/ros2.log"

  echo ">>> 启动单个 GNOME Terminal 窗口，并在其中创建 ${terminal_total} 个标签页"
  gnome-terminal "${terminal_args[@]}"
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
  - 点云输入:    ${POINTCLOUD_TOPIC}

终端布局:       ${TERMINAL_LAYOUT}（tabs=单窗口多标签，windows=多窗口）
Gazebo 界面:    自动启动（gzserver + 独立标签页/终端中的 gzclient）
QGC 界面:       自动启动

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

停止: 关闭终端窗口即可
================================================================
MSG
