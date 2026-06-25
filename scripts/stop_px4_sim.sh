#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
PX4_DIR="${PX4_DIR:-/home/p/PX4-Autopilot}"
QGC_DIR="${QGC_DIR:-/home/p/下载/PX4/qgc-daily-root/squashfs-root}"

echo ">>> 停止所有仿真组件..."

# 0. 关闭 tmux/headless 模式启动的组件
if command -v tmux >/dev/null 2>&1; then
  tmux kill-session -t px4-sim 2>/dev/null || true
fi

if [[ -d "${WS_DIR}/.runtime/logs" ]]; then
  for pid_file in "${WS_DIR}/.runtime/logs"/*.pid; do
    [[ -f "${pid_file}" ]] || continue
    pid="$(cat "${pid_file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
fi

# 1. 杀死 .runtime/ 下的所有包装脚本
for script in start_px4_sitl start_gazebo_gui start_agent wait_for_qgc start_qgc wait_for_fmu start_ros2_autonomy; do
  pkill -f "${WS_DIR}/\.runtime/${script}" 2>/dev/null || true
done

# 2. 杀死 gnome-terminal 窗口（按标题匹配）
for title in "PX4 SITL" "PX4 SITL + Gazebo" "Gazebo GUI" "MicroXRCEAgent" "QGroundControl" "ROS2 Autonomy"; do
  # 先用 wmctrl 优雅关闭（如果存在）
  if command -v wmctrl >/dev/null 2>&1; then
    wmctrl -F -c "${title}" 2>/dev/null || true
  fi
  # 备用：直接 pkill 匹配标题的 gnome-terminal
  pkill -f "gnome-terminal.*--title=${title}" 2>/dev/null || true
  pkill -f "gnome-terminal.*--title=${title}" 2>/dev/null || true
done

# 3. 杀死所有仿真核心进程
pkill -x px4 2>/dev/null || true
pkill -f "${PX4_DIR}/build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -f "px4_sitl_default" 2>/dev/null || true
pkill -f "MicroXRCEAgent udp4 -p" 2>/dev/null || true
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x gazebo 2>/dev/null || true
pkill -f "${QGC_DIR}/AppRun" 2>/dev/null || true
pkill -f "QGroundControl" 2>/dev/null || true
pkill -f "ros2 launch px4_autonomy_bringup" 2>/dev/null || true
pkill -f "ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py" 2>/dev/null || true
pkill -f "ros2 launch nav2_bringup" 2>/dev/null || true
pkill -f "px4_autonomy_mode" 2>/dev/null || true
pkill -f "qgc_reposition_goal_bridge" 2>/dev/null || true
pkill -f "px4_local_position_nav2_bridge" 2>/dev/null || true
pkill -f "ego_planner_node" 2>/dev/null || true
pkill -f "ego_traj_server" 2>/dev/null || true
pkill -f "trajectory_interface" 2>/dev/null || true
pkill -f "octomap_server" 2>/dev/null || true
pkill -f "nav2_" 2>/dev/null || true
pkill -f "gz_scan_to_pointcloud" 2>/dev/null || true
pkill -f "gz_scan_min_distance" 2>/dev/null || true
pkill -f "gz_six_direction_distance" 2>/dev/null || true
pkill -f "lio_odometry_bridge" 2>/dev/null || true
pkill -f "ros_gz_bridge" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true

# 4. 清理临时文件
rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null || true
rm -rf "${WS_DIR}/.runtime" 2>/dev/null || true

echo "全部组件已停止，终端窗口已关闭，临时文件已清理。"
