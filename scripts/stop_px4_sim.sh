#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
PX4_DIR="${PX4_DIR:-/home/p/PX4-Autopilot}"
QGC_DIR="${QGC_DIR:-/home/p/下载/PX4/qgc-daily-root/squashfs-root}"

echo ">>> 停止所有仿真组件..."

terminate_pattern() {
  local signal="$1"
  local pattern="$2"
  pkill "-${signal}" -f "${pattern}" 2>/dev/null || true
}

terminate_exact() {
  local signal="$1"
  local name="$2"
  pkill "-${signal}" -x "${name}" 2>/dev/null || true
}

# 0. 关闭 tmux/headless 模式启动的组件
if command -v tmux >/dev/null 2>&1; then
  tmux kill-session -t px4-sim 2>/dev/null || true
fi

if [[ -d "${WS_DIR}/.runtime/logs" ]]; then
  for pid_file in "${WS_DIR}/.runtime/logs"/*.pid; do
    [[ -f "${pid_file}" ]] || continue
    pid="$(cat "${pid_file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]]; then
      kill -TERM "${pid}" 2>/dev/null || true
      sleep 0.5
      kill -KILL "${pid}" 2>/dev/null || true
    fi
    rm -f "${pid_file}" 2>/dev/null || true
  done
fi

# 1. 杀死 .runtime/ 下的所有包装脚本
for script in start_px4_sitl start_gazebo_gui start_agent wait_for_qgc start_qgc wait_for_fmu start_ros2_autonomy; do
  terminate_pattern TERM "${WS_DIR}/\.runtime/${script}"
done

# 2. 杀死 gnome-terminal 窗口（按标题匹配）
for title in "PX4 SITL" "PX4 SITL + Gazebo" "Gazebo GUI" "MicroXRCEAgent" "QGroundControl" "ROS2 Autonomy"; do
  # 先用 wmctrl 优雅关闭（如果存在）
  if command -v wmctrl >/dev/null 2>&1; then
    wmctrl -F -c "${title}" 2>/dev/null || true
  fi
  # 备用：直接 pkill 匹配标题的 gnome-terminal
  terminate_pattern TERM "gnome-terminal.*--title=${title}"
  terminate_pattern KILL "gnome-terminal.*--title=${title}"
done

# 3. 杀死所有仿真核心进程
for signal in TERM KILL; do
  terminate_pattern "${signal}" "script -q -f -c .*start_px4_sitl\.sh"
  terminate_pattern "${signal}" "script -q -f -c .*start_gazebo_gui\.sh"
  terminate_pattern "${signal}" "script -q -f -c .*start_agent\.sh"
  terminate_exact "${signal}" px4
  terminate_pattern "${signal}" "${PX4_DIR}/build/px4_sitl_default/bin/px4"
  terminate_pattern "${signal}" px4_sitl_default
  terminate_pattern "${signal}" "MicroXRCEAgent udp4 -p"
  terminate_exact "${signal}" MicroXRCEAgent
  terminate_exact "${signal}" gzserver
  terminate_exact "${signal}" gzclient
  terminate_exact "${signal}" gazebo
  terminate_pattern "${signal}" "${QGC_DIR}/AppRun"
  terminate_pattern "${signal}" QGroundControl
  terminate_pattern "${signal}" "/opt/ros/.*/bin/ros2 launch px4_autonomy_bringup autonomy_stack.launch.py"
  terminate_pattern "${signal}" "ros2 launch px4_autonomy_bringup"
  terminate_pattern "${signal}" "ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py"
  terminate_pattern "${signal}" "ros2 launch nav2_bringup"
  terminate_pattern "${signal}" px4_autonomy_mode
  terminate_pattern "${signal}" qgc_reposition_goal_bridge
  terminate_pattern "${signal}" px4_local_position_nav2_bridge
  terminate_pattern "${signal}" ego_planner_node
  terminate_pattern "${signal}" ego_traj_server
  terminate_pattern "${signal}" trajectory_interface
  terminate_pattern "${signal}" octomap_server
  terminate_pattern "${signal}" nav2_
  terminate_pattern "${signal}" gz_scan_to_pointcloud
  terminate_pattern "${signal}" gz_scan_min_distance
  terminate_pattern "${signal}" gz_six_direction_distance
  terminate_pattern "${signal}" mid360_sim_bridge
  terminate_pattern "${signal}" px4_imu_bridge
  terminate_pattern "${signal}" pointcloud_relay
  terminate_pattern "${signal}" fastlio_mapping
  terminate_pattern "${signal}" lio_odometry_bridge
  terminate_pattern "${signal}" ros_gz_bridge
  terminate_pattern "${signal}" "gz sim"
  terminate_pattern "${signal}" "gz model --verbose --spawn-file"
  terminate_pattern "${signal}" "gz topic -e /gazebo/"
  sleep 1
done

if command -v fuser >/dev/null 2>&1; then
  fuser -k 8888/udp 2>/dev/null || true
fi

# 4. 清理临时文件
rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null || true
rm -rf "${WS_DIR}/.runtime" 2>/dev/null || true

echo "全部组件已停止，终端窗口已关闭，临时文件已清理。"
