#!/usr/bin/env bash
set -euo pipefail
source "/home/p/px4_ros2_ws/.runtime/ros2_autonomy_env.sh"
exec ros2 launch px4_autonomy_bringup autonomy_stack.launch.py \
  mission_size_m:=2.5 \
  mission_altitude_m:=2.0 \
  hold_time_s:=3.0 \
  acceptance_radius_m:=0.35 \
  max_horizontal_velocity_m_s:=0.8 \
  max_vertical_velocity_m_s:=0.5 \
  max_heading_rate_deg_s:=60.0 \
  mission_timeout_s:=180.0 \
  auto_rtl_after_finish:=false \
  use_livox:=false \
  launch_obstacle_sim:=false \
  obstacle_sim_mode:=safe \
  obstacle_distance_topic:=/perception/min_obstacle_distance \
  obstacle_stop_distance_m:=5.0 \
  obstacle_abort_distance_m:=1.0 \
  obstacle_hold_timeout_s:=0.0 \
  enable_obstacle_hold:=true \
  target_point_topic:=/autonomy/target_ned \
  accept_runtime_target:=true
