## PX4 GUI Ego-Planner 避障验证流程

本文档记录当前工作区下，`PX4 + Gazebo Classic + Ego Planner` 的一套可复现验证流程，用于确认：

1. PX4 SITL 主链已正常启动
2. 规划与执行链已打通
3. 飞机在目标直线路径存在障碍物时，能够绕飞避障并到达目标点

所有命令默认都应在工作区根目录执行：

```bash
cd /home/p/px4_ros2_ws
```

如果你当前不在工作区根目录，也可以把文中的相对路径改成绝对路径，例如：

```bash
python3 /home/p/px4_ros2_ws/scripts/validate_fastlio_ego_avoidance.py \
  --require-chain \
  --odom-topic /autonomy/lio_odometry \
  --target-x -4.8 \
  --target-y -1.5 \
  --target-z 1.5 \
  --timeout 120
```

注意：

- `ego_planner` 需要先在它自己的 colcon 工作区里重新构建一次，确保 `traj_server` 的 `COMPLETED` 逻辑已经进入运行时产物。
- 这个仓库里的 `check_planning_prereqs.sh` 会检查 `ego_planner` 包是否已在当前环境可见，但不会替你构建上游仓库。

### 1. 冷启动仿真主链（GUI）

先停止所有残留进程：

```bash
./scripts/stop_px4_sim.sh
```

启动带 GUI 的 PX4 SITL。

如果你习惯多窗口：

```bash
TERMINAL_LAYOUT=windows LAUNCH_AUTONOMY_STACK=false ./scripts/start_px4_sim.sh
```

如果你习惯单窗口多标签页：

```bash
TERMINAL_LAYOUT=tabs LAUNCH_AUTONOMY_STACK=false ./scripts/start_px4_sim.sh
```

这两种模式都会启动 GUI 窗口，便于你直接观察避障绕飞过程。

### 2. 检查 PX4 主链

确认 PX4 uXRCE-DDS 话题已正常发布：

```bash
ros2 topic info /fmu/out/vehicle_local_position_v1
ros2 topic info /fmu/out/vehicle_status_v4
```

通过标准：

- `/fmu/out/vehicle_local_position_v1` 的 `Publisher count` 大于 0
- `/fmu/out/vehicle_status_v4` 话题存在

如果失败，优先查看：

```bash
sed -n '1,220p' .runtime/logs/px4.log
sed -n '1,220p' .runtime/logs/gz_spawn.log
```

### 3. 启动规划与执行链

```bash
source /opt/ros/humble/setup.bash
source /home/p/px4_ros2_ws/install/setup.bash

ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py \
  use_sim_time:=false \
  use_fastlio_bridge:=false \
  use_sim_bridge:=true \
  use_depth_camera_fastlio:=false \
  use_ego_planner:=true \
  use_simple_avoidance_fallback:=false \
  use_native_3d_pointcloud:=false \
  use_scan_fallback:=false \
  lio_odom_topic:=/autonomy/lio_odometry \
  planner_odom_topic:=/autonomy/lio_odometry \
  local_map_topic:=/autonomy/local_map \
  native_pointcloud_topic:=/sim/mid360/points \
  depth_cloud_topic:=/sim/mid360/points \
  sim_imu_topic:=/sim/imu \
  fixed_goal_altitude_m:=1.5 \
  max_vel:=0.75 \
  max_acc:=1.0 \
  planning_horizon:=9.0 \
  map_size_x:=30.0 \
  map_size_y:=30.0 \
  map_size_z:=4.0 \
  obstacle_inflation:=0.78 \
  collision_distance:=0.95 \
  collision_weight:=8.0 \
  replan_time_threshold:=0.20 \
  replan_distance_threshold:=0.30 \
  control_points_distance:=0.25 \
  max_jerk:=2.5 \
  ground_height:=0.2 \
  trajectory_auto_arm:=true \
  trajectory_auto_set_offboard:=true
```

### 4. 检查规划链路

确认关键节点与话题都已就绪：

```bash
ros2 node list | grep -E "ego_planner_node|ego_traj_server|trajectory_interface|px4_local_position_ego_bridge|gazebo_depth_to_pointcloud_front"

ros2 topic info /autonomy/lio_odometry
ros2 topic info /planning/position_cmd
```

预期现象：

- 存在 `ego_planner_node`
- 存在 `ego_traj_server`
- 存在 `trajectory_interface`
- 存在 `px4_local_position_ego_bridge`
- `/autonomy/lio_odometry` 有 publisher
- `/planning/position_cmd` 有 publisher 

如果要看实时日志：

```bash
sed -n '1,260p' .runtime/logs/ego_planner_offboard.log
```

### 5. 运行自动避障验证

先跑一个离线逻辑自检，确认跨障目标选择和判据没有回退：

```bash
python3 /home/p/px4_ros2_ws/scripts/validate_fastlio_ego_avoidance.py --self-test
```

默认验证脚本会选择一条穿越中部障碍物的目标线：

```bash
python3 /home/p/px4_ros2_ws/scripts/validate_fastlio_ego_avoidance.py \
  --require-chain \
  --odom-topic /autonomy/lio_odometry \
  --target-x -4.8 \
  --target-y -1.5 \
  --target-z 1.5 \
  --timeout 120
```

如果脚本提示 `Target is already too close to current position`，不要继续在当前姿态上复验。因为这意味着飞机已经停在目标附近，规划器只会输出一个很短的保持位姿命令，看起来像“没有绕障”。此时应先重启仿真，或换一个更远的目标后再测。

也可以显式指定目标，例如：

```bash
python3 /home/p/px4_ros2_ws/scripts/validate_fastlio_ego_avoidance.py \
  --require-chain \
  --odom-topic /autonomy/lio_odometry \
  --target-x -4.8 \
  --target-y -0.8 \
  --target-z 1.5 \
  --timeout 120
```

### 6. 通过标准

重点看脚本输出末尾的这些字段：

- `Target reached threshold.`
- `obstacle_xy_touched False`
- `entered_obstacle_aabb False`
- `path_length_m` 明显大于 `straight_line_distance_m`
- `PASS_CLEARANCE_TEST True`

判定含义：

- `Target reached threshold.`：飞机进入目标阈值
- `obstacle_xy_touched False`：飞行轨迹没有从障碍物平面投影穿过中间障碍
- `entered_obstacle_aabb False`：飞机没有穿入障碍物包围盒
- `path_length_m` 明显大于 `straight_line_distance_m`：路径不是简单直线收敛
- `PASS_CLEARANCE_TEST True`：飞机既绕开了障碍物，也满足了最小安全间隙要求

当前默认要求：

- `required_clearance_m = 0.45`
- `target_threshold = 0.45`

当前这一版 launch 默认参数已经进一步收紧，目标是抑制“先高空大绕行、最后贴着障碍角点回切”的轨迹形态。如果冷启动后仍然只是“能绕过但 clearance 不稳定”，建议按下面顺序继续细调，而不是一次改很多项：

1. 当前推荐先固定 `collision_distance = 0.95`
2. 当前推荐先观察 `obstacle_inflation = 0.78` 是否足以消除贴角路径
3. `virtual_ceil_height`：必要时从 `2.4` 再降到 `2.2`
4. `max_vel`：必要时从 `0.75` 再降到 `0.65`

每次只改一项后重新冷启动复验，避免把“真正起作用的参数”混在一起。

### 7. 失败时如何排查

按以下顺序检查：

1. PX4 主链

```bash
ros2 topic info /fmu/out/vehicle_local_position_v1
ros2 topic info /fmu/out/vehicle_status_v4
```

2. 桥接与执行链

```bash
ros2 topic info /autonomy/lio_odometry
ros2 topic info /planning/position_cmd
```

3. 关键日志

```bash
sed -n '1,260p' .runtime/logs/px4.log
sed -n '1,260p' .runtime/logs/gz_spawn.log
sed -n '1,260p' .runtime/logs/ego_planner_offboard.log
```

### 8. 最短 GUI 复验路径

如果只想快速确认“当前是否还能在 GUI 下绕障到点”，可以直接按下面三步：

```bash
./scripts/stop_px4_sim.sh
TERMINAL_LAYOUT=windows LAUNCH_AUTONOMY_STACK=false ./scripts/start_px4_sim.sh
python3 /home/p/px4_ros2_ws/scripts/validate_fastlio_ego_avoidance.py --require-chain --odom-topic /autonomy/lio_odometry --timeout 120
```

最后确认输出中存在：

```text
Target reached threshold.
obstacle_xy_touched False
entered_obstacle_aabb False
PASS_CLEARANCE_TEST True
```

### 9. GUI 观测说明

#### 9.1 多窗口模式

```bash
./scripts/stop_px4_sim.sh
TERMINAL_LAYOUT=windows LAUNCH_AUTONOMY_STACK=false ./scripts/start_px4_sim.sh
```

#### 9.2 单窗口多标签页模式

```bash
./scripts/stop_px4_sim.sh
TERMINAL_LAYOUT=tabs LAUNCH_AUTONOMY_STACK=false ./scripts/start_px4_sim.sh
```

这两种模式下，脚本都会自动拉起：

- `PX4 SITL + Gazebo`
- `Gazebo GUI`
- `MicroXRCEAgent`
- `QGroundControl`

如果你只想观察 Gazebo，不关心 QGC，可以直接把 QGC 窗口关掉。

#### 9.3 GUI 模式下启动规划链

在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/p/px4_ros2_ws/install/setup.bash

ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py \
  use_sim_time:=false \
  use_fastlio_bridge:=false \
  use_sim_bridge:=true \
  use_depth_camera_fastlio:=false \
  use_ego_planner:=true \
  use_simple_avoidance_fallback:=false \
  use_native_3d_pointcloud:=false \
  use_scan_fallback:=false \
  lio_odom_topic:=/autonomy/lio_odometry \
  planner_odom_topic:=/autonomy/lio_odometry \
  local_map_topic:=/autonomy/local_map \
  native_pointcloud_topic:=/sim/mid360/points \
  depth_cloud_topic:=/sim/mid360/points \
  sim_imu_topic:=/sim/imu \
  fixed_goal_altitude_m:=1.5 \
  max_vel:=0.75 \
  max_acc:=1.0 \
  planning_horizon:=9.0 \
  map_size_x:=30.0 \
  map_size_y:=30.0 \
  map_size_z:=4.0 \
  obstacle_inflation:=0.78 \
  collision_distance:=0.95 \
  collision_weight:=8.0 \
  replan_time_threshold:=0.20 \
  replan_distance_threshold:=0.30 \
  control_points_distance:=0.25 \
  max_jerk:=2.5 \
  ground_height:=0.2 \
  launch_rviz:=true \
  trajectory_auto_arm:=true \
  trajectory_auto_set_offboard:=true
```

#### 9.4 GUI 模式下你该观察什么

建议重点看：

1. 飞机是否从起点升空并进入 Offboard 控制
2. RViz 中蓝色实际轨迹 `Actual Flight Path` 是否持续向目标推进，而不是在障碍前缩成短线
3. 黄色 `Optimal Trajectory` 或橙色 `A Star` 是否从障碍物上侧或下侧绕开，而不是在障碍前终止
4. 红色 `Inflated Obstacles` 是否仍为左右绕飞保留通道；如果膨胀区已经把通路封死，参数仍然过保守
5. 飞机是否始终没有穿过障碍物本体
6. 飞机最终是否稳定到达目标附近

如果 RViz 中只有蓝色轨迹、看不到障碍物，先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/p/px4_ros2_ws/install/setup.bash
ros2 topic echo /autonomy/local_map --once
ros2 topic echo /drone_0_grid/grid_map/occupancy_inflate --once
```

当前推荐优先使用 Gazebo 深度图桥接生成 `/autonomy/local_map`，也就是 `use_native_3d_pointcloud:=false`。因为当前 `mid360` 原生插件虽然在发布 `/sim/mid360/points`，但实际内容仍是空点云。

正常情况下这两个 PointCloud2 都不应是 `width: 0`。如果是空点云，说明当前障碍输入链没有真正喂到规划器，RViz 也就不会显示障碍物。

如果 `/autonomy/local_map` 已经有点，但 `/drone_0_grid/grid_map/occupancy_inflate` 仍然是空的，通常说明 `grid_map` 的可视化截断高度或射线参数没有覆盖当前场景。当前 launch 已补上：

- `grid_map/min_ray_length = 0.1`
- `grid_map/max_ray_length = 20.0`
- `grid_map/visualization_truncate_height = 3.5`
- `grid_map/virtual_ceil_height = 3.2`

这组参数生效后，仍然需要重启规划链一次。

如果你还想同时保留自动判定，可以在 GUI 模式下另开一个终端运行：

```bash
python3 /home/p/px4_ros2_ws/scripts/validate_fastlio_ego_avoidance.py \
  --require-chain \
  --odom-topic /autonomy/lio_odometry \
  --timeout 120
```

### 10. RTL 接管实测（无须手点 QGC）

如果你想验证“QGC/外部 GoTo 已激活后，RTL 是否能让 PX4 真正离开 ROS2 external / Offboard 接管”，可以直接运行：

```bash
bash /home/p/px4_ros2_ws/scripts/validate_qgc_rtl_takeover.sh
```

这个脚本会自动完成：

1. 读取当前 `/fmu/out/vehicle_status_v4`
2. 用 `scripts/simulate_qgc_goto.sh` 发送一次模拟 QGC `DO_REPOSITION`
3. 等待几秒让目标保护逻辑激活
4. 发送 `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH`
5. 再次读取 `/fmu/out/vehicle_status_v4` 并输出摘要

重点关注输出末尾：

- `left_ros2_external True`
- `not_stuck_in_offboard True`
- `offboard_not_accepted True`
- `PASS_RTL_TAKEOVER True`

如果 `PASS_RTL_TAKEOVER` 仍然不是 `True`，再结合下面两处日志一起看：

```bash
grep -n "releasing QGC goal protection\|requested ROS2 external mode" .runtime/logs/ego_planner_offboard.log
grep -n "failsafe\|nav_state" .runtime/logs/ego_validation.log 2>/dev/null || true
```
