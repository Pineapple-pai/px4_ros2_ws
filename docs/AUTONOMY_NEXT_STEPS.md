# ROS2 Autonomy 多航点飞行与避障验证指南

本文档用于记录当前 `PX4 + ROS 2 + FAST-LIO + Nav2/Ego-Planner` 自主飞行链路的最新进展，并指导后续在仿真和实机上继续验证。

---

## 2026-07-08 最新进展

### Ego-Planner 小目标、1.5 m 目标与低速绕障已通过

已完成 `PX4 SITL + Gazebo room_obstacles + FAST-LIO + Ego-Planner + trajectory_interface` 的 0.6 m、1.0 m、1.5 m 目标自动验证，以及一轮低速跨障绕行验证。验证脚本会自动起飞到约 1.5 m，切出 helper Offboard 到 Loiter hold，再发布 Ego 目标，由 trajectory interface 自行切入 Offboard，最后发送 LAND 并等待 disarm。

本轮关键修复：

- `pointcloud_relay` 的近机体 self-filter 改为优先使用 planner `/odom`，确保 FAST-LIO map、Ego odom、目标点处于同一个 world 原点；PX4 local position 只作为后备。
- `pointcloud_self_filter_radius_xy_m` 和 `pointcloud_self_filter_radius_z_m` 默认提高到 `1.00`，减少机体自身、桨盘附近残留点、近场噪声进入 Ego 局部地图。
- `trajectory_interface` 增加 `align_planner_frame_to_px4_local`，在 ROS2 Autonomy/Offboard 交接时记录 planner world 到 PX4 local ENU 的平移 offset，再把 Ego position command 转成 PX4 local ENU setpoint。
- `trajectory_interface` 保留 completed trajectory 后的当前位置 hold，避免 Ego 短目标完成后停止 setpoint 导致 PX4 Offboard signal lost。
- `trajectory_interface` 对每个位置 setpoint 做步长限制：水平默认 `0.35 m`，垂直默认 `0.18 m`。
- LAND/RTL/disarm/非 Offboard mode 命令仍会抑制 trajectory 输出；持久外部接管期间忽略 planner command，避免 LAND 后重新对齐或抢占。
- 验证脚本允许 PX4 冷启动后的地面 `AUTO_LOITER` 初始模式，但仍要求未解锁、接近地面、无 failsafe。
- `EGOReplanFSM::getLocalTarget()` 增加短距离/边界采样防护：未采样到 planning horizon 点时，局部目标必须回退到全局终点 `end_pt_`，避免局部目标沿用默认 `[0,0,0]` 造成下降或返航式错误规划。
- `validate_ego_small_goal_handover.py` 的 LAND 完成判定改为“已 disarm 且高度低于 `0.80 m`”，兼容 PX4 local altitude 在 Gazebo disarm 后的残余漂移。
- `validate_fastlio_ego_avoidance.py` 已改为和小目标脚本一致的安全流程：helper Offboard 起飞、Loiter hold、再发布 Ego 跨障目标；不再从地面直接 ARM 后穿障。

最终复测结果：

```text
0.6 m repeat #1: passed=True, min_goal_dist=0.200, max_move=0.422, max_alt=1.474, LAND_COMPLETE
0.6 m repeat #2: passed=True, min_goal_dist=0.194, max_move=0.439, max_alt=1.497, LAND_COMPLETE
0.6 m repeat #3: passed=True, min_goal_dist=0.197, max_move=0.428, max_alt=1.466, LAND_COMPLETE

1.0 m: passed=True, min_goal_dist=0.197, max_move=0.837, max_alt=1.562, LAND_COMPLETE

1.5 m cold start:
published_small_goal: x=1.369, y=0.211, z=1.549
offboard_seen: true
min_goal_dist: 0.196 m
max_motion_from_start: 1.324 m
max_alt: 1.582 m
failsafe: false
result: SMALL_GOAL_RESULT passed=True bad=none
final: LAND_COMPLETE, disarmed

low-speed obstacle avoidance:
launch params: max_vel=0.25, max_acc=0.35, obstacle_inflation=0.60, collision_distance=0.65, collision_weight=8.0, virtual_ceil_height=2.15
target: current left side -> (1.15, -0.20, 1.50)
frame_alignment: horizontal_error=0.349 m, vertical_error=0.108 m
min_dist_to_mid_obstacle_aabb_m: 1.234
required_clearance_m: 0.35
min_target_dist_m: 0.543
path_length_m: 14.227
straight_line_distance_m: 7.241
max_alt_m: 2.080
entered_obstacle_aabb: False
obstacle_xy_touched: False
crossed_obstacle_side: True
chain_ok: True
result: PASS_CLEARANCE_TEST True
final: LAND_COMPLETE, disarmed

active trajectory LAND takeover:
takeover_after_s: 8.0
takeover_command: land
nav_before_takeover: Offboard
nav_after_takeover: LAND
takeover_nav_seen: True
failsafe: False
result: TAKEOVER_TEST True
final: LAND_COMPLETE, disarmed

physical-route obstacle avoidance, target behind obstacle:
date: 2026-07-08
world/model: room_obstacles + iris_mid360_sim
route intent: obstacle is between start and target; target is behind the mid/high obstacle; aircraft must detour before landing
validation target frame: px4_enu
validation target: (-5.55, -2.20, 1.50)
planner goal after frame offset: (-5.592, -2.075, 1.559)
launch params: max_vel=0.22, max_acc=0.32, obstacle_inflation=0.90, collision_distance=0.80, collision_weight=10.0, virtual_ceil_height=2.20
frame_alignment: horizontal_error=0.132 m, vertical_error=0.059 m
straight_line_crosses_obstacle: True
target_behind_obstacle: True
min_dist_to_mid_obstacle_aabb_m: 0.967
required_clearance_m: 0.35
min_target_dist_m: 0.538
path_length_m: 7.239
straight_line_distance_m: 6.025
target_landing_clearance_m: 2.802
final_landing_clearance_m: 2.559
landing_zone_clearance_required_m: 0.80
entered_obstacle_aabb: False
obstacle_xy_touched: False
crossed_obstacle_side: True
chain_ok: True
result: PASS_CLEARANCE_TEST True
final: LAND_COMPLETE, disarmed
```

小目标/1.5 m 复测中 Ego 日志未再出现：

```text
ERROR! the drone is in obstacle. This should not happen.
First 3 control points in obstacles!
Ran out of pool
Adjusted local target point from [0.000, 0.000, 0.000]
```

低速绕障通过轮次中仍出现过若干 `First 3 control points in obstacles! return false` 重规划警告；物理目标点在障碍物后方的验证轮次中，Ego 日志还出现过 2 次 `ERROR! the drone is in obstacle. This should not happen.`，发生在接近目标点时 planner 将局部起点/目标点上调。该轮 PX4/Gazebo 物理轨迹判据仍通过：没有进入中间障碍 AABB、没有触碰障碍 XY 区域、无 failsafe、最终 LAND_COMPLETE。但这说明 planner 的局部地图/膨胀/目标收敛仍有边界问题，实机前必须继续保守降速、增大安全距离，并优先解决接近目标点时的 Ego 起点/目标修正告警。

当前结论：Ego-Planner 实机同构链路已经从“链路打通但飞行失败”推进到“0.6 m 三连通过、1.0 m 通过、1.5 m 冷启动通过、低速跨障绕行通过、物理目标点在障碍物后方的绕飞验证通过”。现在可以进入更严格的实机前台架/半实物准备，但还不能直接做实机自由绕障飞行。

已完成的下一步链路检查：

```text
validate_fastlio_ego_avoidance.py --no-arm --require-chain
FRAME_ALIGNMENT horizontal_error=0.220 m, vertical_error=0.009 m
goal_publish_count: 1
position_cmd_received: True
bspline_has_publisher: True
position_cmd_has_publisher: True
chain_ok: True
```

下一步执行顺序：

1. 实机前台架检查：MID360 外参、FAST-LIO `/Odometry` 稳定性、PX4 local 与 planner frame offset、RC 手动接管、LAND/RTL/disarm 抑制。
2. 实机首次飞行只允许低速、低高度、短距离目标，并保留人工遥控器 mode switch/kill 兜底。

### 实机台架检查清单

当前环境未连接真实飞控、MID360、遥控器和电机台架，因此实机台架检查尚未实际执行。上机前必须逐项确认：

- 自动检查脚本：`scripts/check_real_bench_readiness.py --duration-s 10`。当前无硬件环境运行结果为 `BENCH_READINESS_RESULT passed=False`，原因是 PX4、LiDAR、FAST-LIO 和 planner topic 均未在线；连接真实台架后必须通过该脚本。
- 机械与安全：拆桨；机体固定；电池固定；急停/断电路径明确；遥控器 mode switch 与 kill switch 可用。
- PX4 通信：`MicroXRCEAgent` 与飞控稳定连接；`/fmu/out/vehicle_status_v4`、`/fmu/out/vehicle_local_position_v1`、`/fmu/out/sensor_combined` 连续发布。
- MID360 与 FAST-LIO：`/livox/lidar`、`/livox/imu` 时间戳单调；FAST-LIO `/Odometry` 静止漂移小；快速轻推机体时姿态/位置无跳变。
- 坐标对齐：悬停/台架静止时 planner `/odom` 与 PX4 local ENU 水平误差建议 `<0.5 m`，垂直误差建议 `<0.3 m`；超过阈值禁止切 Ego Offboard。
- Ego 输入点云：`/autonomy/ego_local_map` 不包含机体自身、桨盘、地面近噪声；self-filter 仍使用 planner `/odom`。
- 外部接管：在 planner 输出活跃时测试 LAND、RTL、disarm、手动 mode 切换；trajectory_interface 必须立即停止抢 setpoint。
- 首飞参数：使用 `max_vel<=0.25`、`max_acc<=0.35`、`virtual_ceil_height` 按场地高度设置；目标距离从 `0.5 m`、`1.0 m` 逐步扩大。

---

## 2026-07-07 最新进展

### Ego-Planner 实机同构链路验证

已进入 `PX4 SITL + Gazebo + FAST-LIO + Ego-Planner + trajectory_interface` 实机同构链路，采用的启动边界为：

```text
PX4/Gazebo/MicroXRCEAgent
FAST-LIO: /livox/lidar + /livox/imu -> /Odometry, /autonomy/local_map
Ego-Planner: /odom + /autonomy/ego_local_map -> /planning/bspline
trajectory_interface: /planning/position_cmd -> /fmu/in/trajectory_setpoint
```

本次已确认通过的链路项：

- FAST-LIO、Ego-Planner、trajectory interface 可在不启动 Nav2/mission controller 的情况下单独运行。
- `/livox/lidar`、`/Odometry`、`/odom`、`/autonomy/local_map`、`/autonomy/ego_local_map` 连续发布。
- 给 `/move_base_simple/goal` 发布短距离目标后，Ego-Planner 能输出 `/planning/bspline` 和 `/planning/position_cmd`。
- trajectory interface 能输出 `/fmu/in/trajectory_setpoint`，PX4 能进入 Offboard。
- 停止 Ego/trajectory 输出后，PX4 可执行 LAND 并最终 disarm。

上一轮 1 m 目标飞行闭环结果：

```text
takeoff_stable: true, altitude ~= 1.88 m
ego_goal: current FAST-LIO odom + 1.0 m, z = 2.0 m
offboard_seen: true
bspline_delta: 103
position_cmd_delta: 2415
trajectory_setpoint_delta: 1210
max_motion_from_start: 4.92 m
failsafe: false during Offboard monitor
final: LAND + disarm
```

新增 0.35 m 小目标飞行闭环结果：

```text
takeoff_stable: true, altitude ~= 1.40 m
hover frame: planner=(0.121, 0.170, 1.458), px4_enu=(0.180, 0.005, 1.397)
frame_error: horizontal=0.175 m, vertical=0.061 m
ego_goal: current FAST-LIO odom + 0.35 m, z=current z
offboard_seen: true
bspline_delta: 19
position_cmd_delta: 19
trajectory_setpoint_delta: 19
min_goal_dist: 0.330 m
max_motion_from_start: 1.306 m
final_before_abort: (0.721, 0.288, 2.612), nav=14, armed=2, failsafe=false
result: FAILED, reason=overshoot
active_trajectory_land: timeout, still Offboard at alt ~= 3.92 m
post_abort: stopped Ego/trajectory output, sent LAND, final alt ~= 0.05 m, disarmed
```

Ego-Planner 日志中的关键异常：

```text
Adjusted start point / local target point upward, e.g. target z 1.458 -> 1.775 -> 1.925 -> 2.075
ERROR! the drone is in obstacle. This should not happen.
First 3 control points in obstacles!
later: Ran out of pool, index=-1 78 127, POOL_SIZE=256 256 128
```

结论：Ego-Planner 链路层已经打通，但飞行闭环明确未通过。当前主因不是简单的 PX4/ROS 坐标不通，而是 Ego 局部地图或膨胀参数让 planner 判断“机体在障碍物中”，随后不断把起点/目标向上修正并导致 Offboard 超调。active trajectory 输出期间 LAND 也会被持续 setpoint 抢占，因此在修复前禁止继续扩大目标或进入实机飞行。

已做的安全收紧：

- `ego_planner_offboard.launch.py` 中 `trajectory_auto_arm` 默认改为 `false`，Ego 规划链默认不再自动解锁。
- `trajectory_interface` 增加 `require_armed_before_offboard`，默认未解锁时不自动请求 Offboard。
- `trajectory_interface` 增加 Offboard/arm 起始门禁：自动切 Offboard/自动 arm 前，必须收到 PX4 local position，并确认 planner `position_cmd` 与 PX4 local ENU 的水平/垂直误差低于阈值。
- `trajectory_interface` 增加外部接管抑制：监听 PX4/MAVLink 侧 LAND、RTL、Disarm、非 Offboard mode 命令，收到后立即挂起 trajectory setpoint 输出，直到 PX4 disarm 后清除。
- `validate_fastlio_ego_avoidance.py` 增加 frame alignment 前置检查：发布飞行目标前，先比较 `/odom` 与 `/fmu/out/vehicle_local_position_v1` 转换后的 ENU 坐标。
- `validate_fastlio_ego_avoidance.py` 增加 flight validation 收尾 LAND：非 `--no-arm` 测试结束后会发送 `NAV_LAND` 并等待 disarm。

新增门禁复测结果：

```text
FRAME_ALIGNMENT horizontal_error=0.010 m, vertical_error=0.031 m
arming_state: 1
nav_state: 4
chain_ok: True
NO_ARM_CHAIN_ONLY
```

这说明 no-arm 链路验证时 planner ENU 与 PX4 local ENU 对齐，且未解锁状态下不会再被 trajectory interface 自动切入 Offboard。

进一步优化：

- `validate_fastlio_ego_avoidance.py --no-arm` 默认改为短链路目标，不再发布穿障远目标。
- no-arm 短链路目标默认约为当前 ENU 位置前方 `0.6 m`，高度为 `max(current_z + 0.3, 0.5)`。
- `ego_planner_offboard.launch.py` 将 `astar_pool_size_x/y/z` 参数显式暴露，默认仍为 `256/256/128`；后续只有在确认内存余量和确实需要更大地图时再调大。

短链路复测结果：

```text
FRAME_ALIGNMENT horizontal_error=0.020 m, vertical_error=0.003 m
Short chain target: current ENU + ~0.6 m, z = 0.5 m
nav_state: 4
chain_ok: True
new Ran out of pool: not observed
```

下一步必须先修复：

- 修 Ego 输入点云/局部地图过滤：禁止把机体自身、近地噪声、桨盘附近点云或过大的 inflation 当成当前机体占据障碍。
- 收紧 Ego 高度边界：先把 `virtual_ceil_height` 和 `ground_height` 调到不会迫使 1.4-1.5 m 起飞高度被向上挤压的范围。
- 降低短目标飞行参数：小目标阶段继续使用 `max_vel<=0.35`、`max_acc<=0.45`，先确认 0.3-0.5 m 闭环不过冲。
- 复测外部接管抑制：Offboard 中发送 LAND 后，`trajectory_interface` 必须停止 setpoint 输出，PX4 必须稳定退出/下降并 disarm。
- 只有 0.35 m 小目标稳定通过后，才扩大到 0.6 m、1.0 m 和绕障目标。

当前已经完成一轮接近实机接口的 headless 仿真闭环：

- PX4 SITL + Gazebo Classic + MicroXRCEAgent 正常启动。
- Gazebo `iris_mid360_sim` 模型输出 32 线 MID360 风格点云。
- `/sim/mid360/points_raw` 通过 `mid360_sim_bridge` 转为 `/livox/lidar`。
- PX4 `/fmu/out/sensor_combined` 通过 `px4_imu_bridge` 转为 `/livox/imu`。
- FAST-LIO 正常输出 `/Odometry`，静止状态约 9.5-10 Hz。
- Nav2 不再直接消费 FAST-LIO 的 `/autonomy/cloud_registered`。
- Nav2/Octomap 改为消费独立轻量点云 `/autonomy/nav2_cloud`，由 `/livox/lidar` 限频抽稀生成。
- 自动起飞到约 2 m 后，成功切入 PX4 ROS2 Autonomy / External1。
- Nav2 成功输出 `/cmd_vel`，PX4 接收 ROS2 Autonomy 控制，最终 LAND 并 disarm。

本轮自动飞行验证结果：

```text
bad: None
seen_external: True
cmd_count: 1060
max_cmd_speed: 0.299 m/s
max_lio_norm: 3.169 m
max_lio_step: 0.077 m
final_rel_alt: 0.047 m
final_arming_state: 1
```

这说明当前 Nav2 固定高度阶段已经可作为安全中间验证链路。最终实机方案仍应转向 Ego-Planner 三维轨迹规划。

---

## 本次代码审查与清理结论

已经清理的无用文件：

- 删除旧 `build/`，随后为恢复 `--symlink-install` 工作区重建了当前必要包
- 删除 `log/`
- 删除 `.runtime/`
- 删除所有 `__pycache__/`
- 删除所有 `*.pyc`

保留的内容：

- 保留 `install/`，因为当前启动脚本会检查 `install/setup.bash`；后续构建会刷新已改包。
- 保留当前重建后的 `build/`，因为 `--symlink-install` 下 `install/` 的开发模式 hook 会引用 `build/` 内文件。
- 保留未跟踪但当前链路需要的功能文件，例如：
  - `src/px4_fastlio_bridge/px4_fastlio_bridge/mid360_sim_bridge.py`
  - `src/px4_obstacle_tools/px4_obstacle_tools/pointcloud_relay.py`
  - `scripts/validate_fastlio_ego_avoidance.py`
  - `scripts/run_ego_planner_validation.sh`
  - `src/px4_gazebo_depth_bridge/`

建议下一次整理 git 时，把这些当前已经参与链路的文件纳入版本管理，而不是删除。

本次审查中确认/修复的关键点：

- `px4_imu_bridge` 增加 IMU 时间戳单调保护，避免 FAST-LIO 因高频时间戳微小倒退而反复 `clear buffer`。
- `pointcloud_relay` 支持限频、抽稀、最大点数限制和 frame 改写，用于隔离 Nav2/Octomap 与 FAST-LIO 主定位链路。
- `lio_odometry_bridge` 增加 FAST-LIO 输出 sanity gate，避免异常定位跳变直接进入控制链路。
- `lio_odometry_bridge` 转发点云时改为复制消息再改 frame，避免修改收到的消息对象。
- `trajectory_interface` 监听 PX4 vehicle command，收到 LAND/RTL/disarm/非 Offboard mode 后挂起 trajectory setpoint 输出，避免降落命令被 active Offboard setpoint 抢占。
- `stop_px4_sim.sh` 补充清理 `mid360_sim_bridge`、`px4_imu_bridge`、`pointcloud_relay`、`fastlio_mapping` 等进程，避免残留进程污染下一次验证。

---

## 当前推荐架构

分阶段推进：

1. Nav2 固定高度验证链路

   用途：确认 PX4 External1、ROS2 控制接管、FAST-LIO 输入、点云避障输入、LAND/disarm 流程都稳定。

   当前状态：已经通过一次自动起飞、切 External1、Nav2 输出速度、自动降落验证。

2. Ego-Planner 三维规划链路

   用途：作为最终实机方案，输入 FAST-LIO odom 和局部点云，输出三维轨迹/位置命令，再由 PX4 trajectory interface 转为 PX4 setpoint。

   当前状态：0.6 m 小目标飞行闭环已通过；下一步做重复性、1.0-1.5 m 无障碍目标和低速绕障验证，再进入实机台架。

最终实机边界应保持：

```text
PX4 飞控
  - EKF/姿态/电机控制/ failsafe / mode 管理
  - 接收 ROS2 External/Offboard setpoint

ROS2 机载计算机
  - MID360 driver -> /livox/lidar
  - PX4 IMU or external IMU -> /livox/imu
  - FAST-LIO -> /Odometry, /cloud_registered
  - Ego-Planner -> 三维轨迹/位置命令
  - trajectory_interface -> /fmu/in/trajectory_setpoint 等 PX4 输入
```

---

## 下一步执行顺序

### Step 1: 重新构建已改包

清理后先刷新相关包：

```bash
cd /home/p/px4_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  px4_fastlio_bridge \
  px4_obstacle_tools \
  px4_autonomy_bringup \
  px4_nav2_bridge \
  px4_trajectory_interface \
  --symlink-install
```

### Step 2: 复测 Nav2 基线

目的：确认清理和小修后，当前基线没有回退。

```bash
cd /home/p/px4_ros2_ws
TERMINAL_LAYOUT=headless \
KEEP_TERMINALS_OPEN=false \
ENABLE_OBSTACLE_AVOIDANCE=false \
USE_FASTLIO=true \
USE_LIVOX=false \
USE_NAV2=true \
NAV2_ODOM_SOURCE=px4_local \
LAUNCH_OBSTACLE_SIM=false \
LAUNCH_GZ_SCAN_DISTANCE=false \
LAUNCH_GZ_SIX_DIRECTION_DISTANCE=false \
PX4_WORLD=room_obstacles \
PX4_MODEL=iris_mid360_sim \
FASTLIO_RVIZ=false \
./scripts/start_px4_sim.sh
```

起飞前必须看到：

```text
/livox/lidar              约 10 Hz
/livox/imu                高频且无 timestamp back
/Odometry                 约 9.5-10 Hz
/autonomy/nav2_cloud      约 4-5 Hz, 约 5000 点
PX4 pre_flight_checks_pass true
```

### Step 3: 转入 Ego-Planner 实机同构链路验证

目标：验证最终实机同构链路，而不是继续扩展 Nav2。

当前只做 chain validation，不再直接做 flight validation。先启动 PX4/Gazebo/Agent，不启动 Nav2/Autonomy stack：

```bash
cd /home/p/px4_ros2_ws
TERMINAL_LAYOUT=headless \
KEEP_TERMINALS_OPEN=false \
LAUNCH_AUTONOMY_STACK=false \
ENABLE_OBSTACLE_AVOIDANCE=false \
PX4_WORLD=room_obstacles \
PX4_MODEL=iris_mid360_sim \
FASTLIO_RVIZ=false \
./scripts/start_px4_sim.sh
```

再单独启动 FAST-LIO：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_fastlio_bridge fastlio_mapping.launch.py \
  use_livox:=false \
  use_sim_time:=false \
  fastlio_config_file:=mid360.yaml \
  rviz:=false \
  publish_nav2_tf:=false \
  nav2_map_frame_id:=world \
  nav2_odom_frame_id:=odom \
  nav2_base_frame_id:=base_link
```

再单独启动 Ego-Planner/trajectory interface：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py \
  use_sim_time:=false \
  use_fastlio_bridge:=false \
  use_sim_bridge:=false \
  use_depth_camera_fastlio:=false \
  use_ego_planner:=true \
  use_simple_avoidance_fallback:=false \
  use_native_3d_pointcloud:=true \
  native_pointcloud_topic:=/autonomy/local_map \
  local_map_topic:=/autonomy/ego_local_map \
  planner_odom_topic:=/odom \
  lio_odom_topic:=/autonomy/lio_odometry \
  fixed_goal_altitude_m:=1.5 \
  launch_rviz:=false \
  max_vel:=0.35 \
  max_acc:=0.45 \
  planning_horizon:=4.0 \
  trajectory_auto_arm:=false \
  trajectory_auto_set_offboard:=true \
  suspend_on_external_mode_command:=true \
  require_armed_before_offboard:=true \
  require_local_position_before_offboard:=true \
  max_offboard_start_horizontal_error_m:=0.8 \
  max_offboard_start_vertical_error_m:=0.5 \
  align_planner_frame_to_px4_local:=true \
  pointcloud_self_filter_radius_xy_m:=1.0 \
  pointcloud_self_filter_radius_z_m:=1.0 \
  astar_pool_size_x:=256 \
  astar_pool_size_y:=256 \
  astar_pool_size_z:=128
```

先做不解锁链路验证：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_ego_planner_validation.sh
```

`VALIDATION_MODE=chain` 或默认模式下，脚本内部会使用 `--no-arm`，只发短目标做链路验证；不要在这一步做穿障飞行。

验证标准：

- FAST-LIO `/Odometry` 持续稳定。
- Ego-Planner 收到 odom 和局部点云。
- `FRAME_ALIGNMENT` 输出中 planner ENU 与 PX4 local ENU 误差低于阈值。
- trajectory interface 输出 PX4 setpoint。
- 不要求解锁、不要求起飞、不要求进入 Offboard。

再做 0.6 m 小目标飞行验证：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -u scripts/validate_ego_small_goal_handover.py \
  --goal-forward-m 0.6 \
  --max-move-pass-m 1.2
```

飞行验证标准：

- 脚本自动起飞、切 Loiter handover、发布 Ego 小目标、进入 Offboard、LAND/disarm。
- `SMALL_GOAL_RESULT passed=True bad=none`。
- `failsafe=False`，`offboard_seen=True`。
- planner 日志中不得再出现 `drone is in obstacle`、`First 3 control points in obstacles`、`Ran out of pool`。
- Offboard 中发送 LAND/RTL/Disarm 或切非 Offboard mode 时，`trajectory_interface` 必须立即挂起 trajectory setpoint 输出。
- 通过后再扩大到 1.0 m、1.5 m 和低速绕障目标。

### Step 4: 实机前台架检查

实机前不要直接飞，先上台架或拆桨验证：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic echo --once /fmu/out/vehicle_status_v4
ros2 topic echo --once /fmu/out/vehicle_local_position_v1
```

必须确认：

- MID360 点云稳定。
- IMU 时间戳单调。
- FAST-LIO 静止不发散。
- PX4 mode 切换命令可用。
- failsafe 策略明确，遥控器/地面站可以随时接管。

### Step 5: 实机低风险首飞

首飞只做最小闭环：

1. 手动 Position 起飞到 1.5-2.0 m。
2. 悬停 10 s，确认 FAST-LIO 不漂。
3. 切 ROS2 External/Offboard。
4. 只给一个 0.5-1.0 m 的短距离目标。
5. 成功后立即 LAND。
6. 分析 PX4 log、ROS bag、FAST-LIO 轨迹。

---

## 旧版多航点/Nav2说明

以下内容保留用于 Nav2 固定高度阶段和多航点任务参考。后续实机主线以 Ego-Planner 三维规划为准。

---

## 🚀 极简快速入门：HTML 页面点击生成航点 → 自动加载执行

本项目的可视化航点编辑器 (`tools/waypoint_editor.html`) 支持**点击画布生成航点 → 导出 YAML 文件 → 仿真启动时自动加载**的完整闭环：

```
┌──────────────────────────────────────────────────────────────────┐
│  ① 打开编辑器（浏览器）                                           │
│     cd /home/p/px4_ros2_ws                                       │
│     python3 -m http.server 8080 &                                │
│     → 浏览器访问 http://localhost:8080/tools/waypoint_editor.html │
├──────────────────────────────────────────────────────────────────┤
│  ② 在画布上点击 → 每点击一次生成一个航点（绿色圆点）              │
│  ③ 点击「💾 下载文件」→ 浏览器下载 mission_waypoints.yaml          │
├──────────────────────────────────────────────────────────────────┤
│  ④ 移动到 missions/（必须用绝对路径）                             │
│     ls -l ~/下载/mission_waypoints.yaml                          │
│     mv ~/下载/mission_waypoints.yaml                             │
│        /home/p/px4_ros2_ws/missions/my_mission.yaml              │
├──────────────────────────────────────────────────────────────────┤
│  ⑤ 启动仿真，YAML 自动加载                                       │
│     cd /home/p/px4_ros2_ws                                       │
│     MISSION_FILE=missions/my_mission.yaml ./scripts/start_px4_sim.sh│
├──────────────────────────────────────────────────────────────────┤
│  ⑥ （可选）回到编辑器点「🔗 连接 rosbridge」实时观察飞行          │
└──────────────────────────────────────────────────────────────────┘
```

> 💡 **整个流程无需写一行代码、无需手动输入坐标**。全部在浏览器中点击完成，导出 YAML 后仿真自动读取执行。

对应的完整终端命令操作：

```bash
# ① 启动 HTTP 服务（如果还没启动）
cd /home/p/px4_ros2_ws
python3 -m http.server 8080 &

# ② 打开编辑器（浏览器）
xdg-open http://localhost:8080/tools/waypoint_editor.html

# ③ 在画布上点击添加航点，然后点击「下载文件」
# ④ 将下载的文件移到 missions/（务必先确认文件存在，再用绝对路径）
cd /home/p/px4_ros2_ws
ls -l ~/下载/mission_waypoints.yaml          # 中文系统用 ~/下载/
# 或 ls -l ~/Downloads/mission_waypoints.yaml  # 英文系统用 ~/Downloads/
mv ~/下载/mission_waypoints.yaml /home/p/px4_ros2_ws/missions/my_mission.yaml
# 如果是英文系统：
# mv ~/Downloads/mission_waypoints.yaml /home/p/px4_ros2_ws/missions/my_mission.yaml

# ⑤ 启动仿真，自动加载航线
cd /home/p/px4_ros2_ws
MISSION_FILE=missions/my_mission.yaml ./scripts/start_px4_sim.sh

# ⑥ 回到编辑器，点击「连接 rosbridge」观察飞行
```

---

## 当前能力边界

- `ROS2 Autonomy` 模式已经支持多航点航线飞行。
- 航线执行过程中，如果激光雷达检测到前方障碍，会规划临时绕障点并尝试侧向绕行。
- 绕障完成后，会回到当前任务航点继续飞行。
- QGC/PX4 原生 `Mission` 模式目前不使用这套 ROS2 绕障逻辑。推荐先用 QGC 起飞、监控和应急接管，航线执行交给 `ROS2 Autonomy`。

## 坐标系说明

多航点使用 PX4 本地 NED 坐标，单位是米：

- `x`: 本地北向/仿真局部 X 方向
- `y`: 本地东向/仿真局部 Y 方向
- `z`: 向下为正，所以飞到 1.5 米高度应写 `-1.5`

示例航点：

```text
0 0 -1.5
2 0 -1.5
2 2 -1.5
0 2 -1.5
```

表示飞机在 1.5 米高度执行一个 2 米见方的多航点任务。

## 快速开始（三步走）

推荐按这个最直接的顺序操作：

### 场景一：先规划航点，再启动仿真（推荐新手）

> **操作顺序**：先打开编辑器点击规划航点 → 导出 YAML → 再启动仿真自动执行

```
① 打开编辑器（浏览器中），点击添加航点
② 点击「下载文件」
③ 必须用绝对路径移动：
    mv ~/下载/mission_waypoints.yaml /home/p/px4_ros2_ws/missions/my_mission.yaml
    或 mv ~/Downloads/mission_waypoints.yaml /home/p/px4_ros2_ws/missions/my_mission.yaml
④ cd /home/p/px4_ros2_ws && MISSION_FILE=missions/my_mission.yaml ./scripts/start_px4_sim.sh
→ 仿真启动后自动加载 YAML 航点 → 飞机自主飞行
⑤ 编辑器中点击「连接 rosbridge」→ 实时观察飞机沿航线飞行
```

对应的终端命令：

```bash
# 第一步：打开编辑器
cd /home/p/px4_ros2_ws
python3 -m http.server 8080 &
xdg-open http://localhost:8080/tools/waypoint_editor.html

# 第二步：在画布上点击添加航点，然后点击「下载文件」下载 YAML
# 将下载的文件移到 missions/ 目录（必须用绝对路径！）
cd /home/p/px4_ros2_ws
ls -l ~/下载/mission_waypoints.yaml && mv ~/下载/mission_waypoints.yaml /home/p/px4_ros2_ws/missions/my_mission.yaml
# 如果是英文系统，用：
# ls -l ~/Downloads/mission_waypoints.yaml && mv ~/Downloads/mission_waypoints.yaml /home/p/px4_ros2_ws/missions/my_mission.yaml

# 第三步：启动仿真，自动加载航线
MISSION_FILE=missions/my_mission.yaml ./scripts/start_px4_sim.sh

# 第四步：回到编辑器，点击「🔗 连接 rosbridge」观察飞行
```

### 场景二：先启动仿真，边看飞机位置边规划（适合调试）

> **操作顺序**：先启动仿真让飞机飞起来 → 打开编辑器连 rosbridge → 根据飞机实时位置点击添加航点 → 导出 → 重启

```
① 先启动仿真：./scripts/start_px4_sim.sh
② 在 QGC 中解锁起飞（Position → Arm → Takeoff → Offboard）
③ 打开编辑器，点击「连接 rosbridge」
   → 画布上出现蓝色十字（飞机实时位置）
④ 根据飞机当前位置，在周围点击添加航点
⑤ 点击「导出 YAML」或「下载文件」，保存到 missions/
⑥ 退出仿真：./scripts/stop_px4_sim.sh
⑦ 用 YAML 文件重新启动：MISSION_FILE=missions/my_mission.yaml ./scripts/start_px4_sim.sh
   → 新航线自动加载执行
```

---

## 完整工作流细节

## 使用可视化编辑器生成航点（推荐）

项目提供了一个基于 HTML 的可视化航点编辑器，支持 **rosbridge 实时联动**：连接后可在画布上实时显示飞机在 Gazebo 仿真中的当前位置（蓝色十字标记），以及在地图网格上**点击创建航点**。

### 启动编辑器

```bash
# 方式一：直接在浏览器中打开（桌面环境）
open tools/waypoint_editor.html

# 方式二：使用 python 启动简易 HTTP 服务（支持 WebSocket）
cd /home/p/px4_ros2_ws
python3 -m http.server 8080
# 然后在浏览器中访问 http://localhost:8080/tools/waypoint_editor.html
```

> **提示**：推荐使用 HTTP 方式启动（方式二），因为编辑器需要 JavaScript 模块加载，部分浏览器对 `file://` 协议有限制。

### 连接 rosbridge（实时显示飞机位置）

启动仿真后，确保 `rosbridge_server` 已运行（`start_px4_sim.sh` 自动启动）：

```bash
# 检查 rosbridge 是否运行
ros2 node list | grep rosbridge
```

在编辑器页面中：

1. 点击右上角 **「🔗 连接 rosbridge」** 按钮
2. 连接成功后，状态指示灯变为绿色 🟢「已连接」
3. 画布上实时显示飞机当前位置（蓝色十字 + 标签）
4. 飞机移动时位置自动更新，可观察航线执行情况

> rosbridge 默认连接地址为 `ws://localhost:9090`，与仿真中 `rosbridge_server` 默认端口一致。

### 编辑器功能

- **点击添加航点**：在画布上点击即可添加航点（点击已有航点可删除）
- **鼠标中键/右键拖拽平移**：按住中键或右键拖拽平移画布
- **滚轮缩放**：滚动滚轮缩放视图
- **航点列表**：右侧面板显示航点列表，支持上移、下移、删除
- **参数设置**：可设置飞行高度、速度、接受半径、避障参数等
- **工具栏**：网格尺寸调节、缩放滑块、复位视角、清空航点
- **实时飞机位置**：连接 rosbridge 后，蓝色十字标记实时显示飞机位置

### 典型工作流：从点击航点到自动飞行

```
┌─ Step 1: 规划航点 ─────────────────────────────────────────────┐
│                                                                 │
│  ① 打开 tools/waypoint_editor.html                              │
│  ② 在画布上点击 → 每点击一次生成一个航点（绿色圆点）           │
│  ③ 拖拽调整位置、右键点击可删除                                │
│  ④ 右侧面板调节：高度、速度、避障参数                          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─ Step 2: 保存 YAML 文件 ───────────────────────────────────────┐
│                                                                 │
│  ① 点击「下载文件」按钮 → 浏览器下载 mission_waypoints.yaml     │
│  ② 将文件移到 workspace 的 missions/ 目录（必须用绝对路径）：   │
│     mv ~/下载/mission_waypoints.yaml                           │
│        /home/p/px4_ros2_ws/missions/my_path.yaml               │
│                                                                 │
│  * 也可以直接点击「导出 YAML」然后复制粘贴到 YAML 文件          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─ Step 3: 启动仿真自动加载 ─────────────────────────────────────┐
│                                                                 │
│  MISSION_FILE=missions/my_path.yaml ./scripts/start_px4_sim.sh  │
│                                                                 │
│  start_px4_sim.sh 检测到 MISSION_FILE 变量后，会自动：           │
│    • 将文件路径传给 ROS2 Autonomy 节点                           │
│    • 节点启动时解析 YAML，加载航点列表                           │
│    • 进入 Offboard 模式后自动按航点飞行                          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─ Step 4: 连接 rosbridge 观察 ──────────────────────────────────┐
│                                                                 │
│  ① 在编辑器中点击「🔗 连接 rosbridge」                         │
│  ② 画布上出现蓝色十字 → 实时显示飞机位置                       │
│  ③ 飞机飞行时位置实时更新 → 验证航线执行情况                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 导出航线文件

编辑器支持三种导出方式：

1. **下载文件** — 点击「下载文件」按钮，直接下载 `mission_waypoints.yaml`
   - 下载后用绝对路径移到 `missions/`：
     ```bash
     mv ~/下载/mission_waypoints.yaml /home/p/px4_ros2_ws/missions/my_mission.yaml
     ```
   - 或英文系统：`mv ~/Downloads/mission_waypoints.yaml /home/p/px4_ros2_ws/missions/my_mission.yaml`
2. **导出 YAML** — 点击「导出 YAML」按钮，生成标准 YAML 格式航线文件（可复制到文本编辑器保存）
3. **导出 Env** — 点击「导出 Env」按钮，生成兼容的 `.env` 格式文件

导出后用法见下方「使用 YAML 航线文件启动」。

---

## 写一个多航点航线文件

推荐两种方式：使用可视化编辑器生成（见上方），或手动创建文件。

### 方式一：YAML 格式（推荐）

YAML 格式结构清晰，支持完整的参数配置，推荐用于长期保存航线。

新建文件：

```bash
mkdir -p missions
nano missions/room_square.yaml
```

写入内容：

```yaml
# PX4 ROS2 Autonomy 多航点航线文件
mission_altitude_m: 1.5
max_horizontal_velocity_m_s: 0.5
acceptance_radius_m: 0.35
mission_timeout_s: 180

enable_local_avoidance: true
avoidance_trigger_distance_m: 3.5
avoidance_clearance_m: 2.0
avoidance_lateral_offset_m: 1.5
avoidance_forward_offset_m: 2.0
avoidance_speed_m_s: 0.35

waypoints:
  - {x: 0.00, y: 0.00, z: -1.50}
  - {x: 2.00, y: 0.00, z: -1.50}
  - {x: 2.00, y: 2.00, z: -1.50}
  - {x: 0.00, y: 2.00, z: -1.50}
  - {x: 0.00, y: 0.00, z: -1.50}
```

这就是可视化编辑器导出的标准 YAML 格式，可直接使用 `mission_file` 参数加载。

### 方式二：.env 格式（传统方式）

```bash
mkdir -p missions
nano missions/room_square.env
```

写入内容：

```bash
MISSION_WAYPOINTS_NED='0 0 -1.5; 2 0 -1.5; 2 2 -1.5; 0 2 -1.5; 0 0 -1.5'
MISSION_ALTITUDE_M=1.5
MAX_HORIZONTAL_VELOCITY_M_S=0.5
MISSION_TIMEOUT_S=180
ACCEPTANCE_RADIUS_M=0.35

ENABLE_LOCAL_AVOIDANCE=true
AVOIDANCE_TRIGGER_DISTANCE_M=3.5
AVOIDANCE_CLEARANCE_M=1.5
AVOIDANCE_LATERAL_OFFSET_M=1.2
AVOIDANCE_FORWARD_OFFSET_M=1.6
AVOIDANCE_SPEED_M_S=0.35
```

航点格式要求：

```text
x y z; x y z; x y z
```

也可以写成逗号分隔：

```bash
MISSION_WAYPOINTS_NED='0,0,-1.5; 2,0,-1.5; 2,2,-1.5; 0,2,-1.5'
```

## 使用航线文件启动仿真

### 方式一：使用 YAML 文件（推荐）

通过 `MISSION_FILE` 环境变量指定 YAML 航线文件（支持相对路径）：

```bash
cd /home/p/px4_ros2_ws
MISSION_FILE=missions/room_square.yaml PX4_WORLD=room_obstacles ./scripts/start_px4_sim.sh
```

也支持绝对路径：

```bash
MISSION_FILE=$(pwd)/missions/room_square.yaml PX4_WORLD=room_obstacles ./scripts/start_px4_sim.sh
```

YAML 航线文件由可视化编辑器一键导出（点击「下载文件」或「导出 YAML」），或手动编写（格式见上方）。

### 方式二：使用 .env 文件（传统方式）

```bash
cd /home/p/px4_ros2_ws
source missions/room_square.env
PX4_WORLD=room_obstacles ./scripts/start_px4_sim.sh
```

启动后按推荐流程操作：

1. 等待 Gazebo、PX4、QGC、ROS2 Autonomy 全部启动。
2. 在 QGC 中使用 `Position` 模式解锁并起飞。
3. 飞机稳定悬停后切换到 `ROS2 Autonomy`。
4. 观察飞机是否按多航点航线飞行。
5. 航线上有障碍时，观察是否减速并规划临时绕障点。
6. 测试结束后切回 `Position`、`Land` 或 `RTL`。

停止仿真：

```bash
./scripts/stop_px4_sim.sh
```

## 飞行中替换多航点航线

也可以不重启仿真，在 `ROS2 Autonomy` 运行时发布新的航点队列：

```bash
source /opt/ros/humble/setup.bash
source /home/p/px4_ros2_ws/install/setup.bash

ros2 topic pub --once /autonomy/waypoints_ned geometry_msgs/msg/PoseArray "
poses:
- position: {x: 0.0, y: 0.0, z: -1.5}
- position: {x: 2.0, y: 0.0, z: -1.5}
- position: {x: 2.0, y: 2.0, z: -1.5}
- position: {x: 0.0, y: 2.0, z: -1.5}
- position: {x: 0.0, y: 0.0, z: -1.5}
"
```

发布成功后，当前航线会被替换，航点索引从第一个航点重新开始。

## 验证多航点是否加载成功

启动后查看 ROS2 日志：

```bash
rg -n "Loaded .* configured mission waypoints|Loaded .* from mission_file|Autonomy mode activated|Heading to waypoint|Reached waypoint" .runtime/logs/ros2.log
```

期望看到类似日志：

```text
Loaded 5 configured mission waypoints from mission_file 'missions/room_square.yaml'.
Autonomy mode activated. waypoints=5 ...
Heading to waypoint 1/5 -> [...]
Reached waypoint 1
Heading to waypoint 2/5 -> [...]
```

使用 YAML 文件时看到 `from mission_file`，使用 `.env` 方式时看到 `from mission_waypoints_ned`。

如果没有看到 `Loaded ... configured mission waypoints`，说明：
- YAML 方式：`mission_file` 路径不正确或文件格式有误
- `.env` 方式：环境变量没有被正确 source，或 `MISSION_WAYPOINTS_NED` 为空

## 验证激光雷达和障碍物距离

查看六向距离：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /perception/six_direction_distance --once
```

期望看到类似：

```text
前: 3.28 m
后: 20.00 m
左: 0.97 m
右: 20.00 m
上: 6.00 m
下: 0.00 m
```

查看 PX4 是否收到 `ObstacleDistance`：

```bash
ros2 topic echo /fmu/in/obstacle_distance --once
```

只要该话题持续有数据，说明 ROS2 感知链路正在向 PX4 输入障碍物距离。

## 验证局部绕行避障

启动带障碍物世界：

```bash
source missions/room_square.env
PX4_WORLD=room_obstacles ./scripts/start_px4_sim.sh
```

飞行中查看日志：

```bash
rg -n "Local avoidance planned|Avoidance waypoint reached|Forward obstacle|Obstacle hold|Reached waypoint" .runtime/logs/ros2.log
```

期望看到：

```text
Local avoidance planned: front ... side=left/right, target=[...]
Avoidance waypoint reached. Returning to mission waypoint ...
Reached waypoint ...
```

这表示飞机在航线遇到障碍时，已经生成临时绕障点，并在绕障后继续执行原航线。

同时采样飞机位置：

```bash
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

关注 `x`、`y`、`z` 是否沿航线变化，遇到障碍时 `x/y` 是否出现侧向绕行轨迹。

## 推荐的验证顺序

1. 先在无障碍或空旷场景验证多航点飞行。
2. 再在 `room_obstacles` 中验证航线上有障碍时是否触发绕行。
3. 降低速度，建议先用 `MAX_HORIZONTAL_VELOCITY_M_S=0.5`。
4. 缩短航线，建议先用 2 到 5 米的小范围任务。
5. 每次测试都保存关键日志：`ros2.log`、`px4.log`、位置采样和六向距离采样。

## 实机前必须补齐的检查

- 激光雷达安装方向和坐标系必须确认。
- `/perception/front_obstacle_distance`、`left`、`right` 必须和真实机体方向一致。
- 第一次实机测试应低速、低高度、开阔环境，旁边必须有人准备遥控器接管。
- 不建议第一次实机就测试绕障穿越窄通道。
- 必须保留 QGC/遥控器切回 `Position`、`Land` 或 `RTL` 的能力。

## 后续建议

下一步可以继续增强：

- ✅ ~~支持从 YAML 航线文件直接加载多航点。~~（已实现，通过 `mission_file` 参数）
- 支持从 QGC mission 导入航点，再由 `ROS2 Autonomy` 执行。
- 支持在编辑器中选择从已有 YAML 文件导入航点进行可视化修改。
- 增加绕障后的回归航线策略，避免在障碍附近反复规划。
- 增加实机传感器健康检查，激光雷达无数据时禁止进入自主航线。

---

## 2026-06-29 EGO Planner 真链路联调建议

针对当前 `FAST-LIO + EGO Planner + px4_trajectory_interface` 联调，建议后续不要继续混用 `px4_autonomy_bringup` 里的 mission 控制链与 `/move_base_simple/goal` 路线。

### 建议执行顺序

1. 关闭当前 `px4_autonomy_bringup` 自带 mission 控制链
   - 如果通过 bringup 启动，增加：`launch_mission_control:=false`
2. 单独启动 EGO Planner 规划链
   - 入口：`ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py`
   - 显式参数：`use_ego_planner:=true`
   - 显式参数：`use_simple_avoidance_fallback:=false`
3. 先验证完整闭环
   - `/map`
   - `/odom`
   - `/move_base_simple/goal`
   - `/planning/bspline`
   - `/planning/position_cmd`
4. 闭环通过后，再重跑同一个穿障目标测试

### 推荐命令

```bash
# 启动规划链
./scripts/start_autonomy_with_planning.sh

# 只检查闭环，不起飞
./scripts/run_ego_planner_validation.sh

# 闭环通过后，再执行飞行穿障验证
VALIDATION_MODE=flight ./scripts/run_ego_planner_validation.sh
```

### 当前结论

- 本轮尚未收敛到“真正稳定绕过障碍并到达目标点”
- 但问题已经从“链路不通”收敛到“fallback 规划器能力不足”
- 下一轮建议直接沿 EGO Planner 真链路继续修正
