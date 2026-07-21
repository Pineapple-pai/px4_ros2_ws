# PX4 ROS 2 自主飞行项目进度与下一步

更新时间：2026-07-10

## 1. 项目目标

当前主线是构建以下三维自主飞行链路：

```text
Gazebo/真实 MID360
  -> /livox/lidar + /livox/imu
  -> FAST-LIO
  -> /Odometry + 局部点云地图
  -> Ego-Planner
  -> /planning/bspline + /planning/position_cmd
  -> trajectory_interface
  -> PX4 Offboard trajectory setpoint
```

现阶段只做 `PX4 SITL + Gazebo` 仿真，不接入真机，不继续增加新功能。重点是提高现有链路的安全性、重复性和可复现性。

## 2. 当前状态

### 2.1 已打通的链路

- PX4 SITL、Gazebo Classic 和 MicroXRCEAgent 可正常启动。
- `iris_mid360_sim` 可发布 MID360 风格仿真点云。
- 仿真点云和 PX4 IMU 可桥接到：
  - `/livox/lidar`
  - `/livox/imu`
- FAST-LIO 可稳定输出：
  - `/Odometry`
  - `/odom`
  - `/autonomy/local_map`
- Ego-Planner 可接收 `/odom` 和 `/autonomy/ego_local_map`，并输出：
  - `/planning/bspline`
  - `/planning/position_cmd`
- `trajectory_interface` 可将规划命令转换为 PX4 trajectory setpoint。
- LAND、RTL、disarm 和非 Offboard mode 命令可抑制 trajectory 输出，避免规划链持续抢占控制权。

### 2.2 已完成的安全修复

- 规划链默认不自动解锁：`trajectory_auto_arm=false`。
- 自动请求 Offboard 前要求 PX4 已解锁，并检查 local position 与 planner frame 对齐。
- planner world 到 PX4 local ENU 支持平移 offset 对齐。
- trajectory setpoint 增加水平和垂直步长限制。
- completed trajectory 后保持当前位置，避免 Offboard setpoint 中断。
- 点云 self-filter 优先使用 planner `/odom`，减少机体和近场噪声进入 Ego 地图。
- Ego 短目标未采样到 planning horizon 点时回退到全局终点，避免使用默认 `[0,0,0]`。
- Ego traj server 可正确发布 trajectory completed 状态。
- 默认回归参数已收紧为：

```text
max_vel=0.25 m/s
max_acc=0.35 m/s²
planning_horizon=4.0 m
trajectory_auto_arm=false
```

### 2.3 已通过的历史仿真测试

- 0.6 m 小目标三次通过。
- 1.0 m 小目标通过。
- 1.5 m 冷启动小目标通过。
- 低速跨障绕行通过。
- 目标位于障碍物后方的物理绕行通过。
- active trajectory 中 LAND 接管通过。
- 测试结束后均可 LAND 并 disarm。

这些历史结果说明主链具备基本飞行能力，但新的严格回归标准高于旧标准，因此必须按后文顺序重新验证。

## 3. 当前严格回归结果

### 3.1 No-arm chain 三连：通过

chain validation 已收紧，不再只检查 topic publisher 是否存在。现在必须在本轮测试中实际收到新的：

- `/planning/bspline`
- `/planning/position_cmd`

同时，每轮使用不同短目标，避免 Ego 将重复目标去重。

严格三连结果：

```text
repeat #1: bspline_received=True, position_cmd_received=True, chain_ok=True
repeat #2: bspline_received=True, position_cmd_received=True, chain_ok=True
repeat #3: bspline_received=True, position_cmd_received=True, chain_ok=True

horizontal frame error: 0.021 / 0.031 / 0.041 m
vertical frame error:   0.025 / 0.025 / 0.029 m
dangerous planner log count: 0
```

结论：不解锁规划链当前稳定通过。

### 3.2 0.6 m small 严格回归：尚未通过

chain 回归结束后直接运行 small profile，首次起飞失败：

```text
helper sent ARM
helper requested OFFBOARD hover takeoff
takeoff_helper_timeout alt=0.07 nav=14 armed=2
```

已定位原因：chain 测试结束后，Ego/trajectory interface 仍保留上一条 completed trajectory 的 hold setpoint。PX4 解锁后，该残留 setpoint 与 small 脚本的 helper 起飞 setpoint 同时发布到 `/fmu/in/trajectory_setpoint`，导致起飞目标被覆盖。

当前结论：

- 不是速度或加速度参数不足。
- chain 与 flight profile 之间必须清除规划执行链的历史 trajectory 状态。
- 起飞超时后必须主动 LAND 并等待 disarm，不能遗留 armed Offboard。
- 失败后的 PX4 若未恢复允许的 nav state 和 preflight 状态，必须冷启动后再测。

## 4. 当前已知问题

### P0：测试之间存在状态污染

chain、small、avoidance 等 profile 共用同一规划进程时，上一轮 trajectory hold 可能影响下一轮 helper 起飞。

需要修复：

- flight profile 前重启 Ego-Planner 和 trajectory interface；或
- 提供可靠的 trajectory reset/suspend 流程；
- 每轮失败必须执行 LAND/disarm 和状态检查；
- 正式回归优先采用每轮冷启动或至少重启规划执行链。

### P0：small 脚本失败收尾不完整

当前 helper 起飞超时后可能直接退出，留下 armed Offboard。

需要修复：任何 ARM 之后的失败路径都必须进入统一 LAND/disarm 收尾逻辑。

### 2026-07-10 本轮 P0 修复进展

已完成代码修复（仍需在 SITL 中执行下述验收）：

- `trajectory_interface` 在 PX4 disarmed 时不再把 planner command 转换为可执行 setpoint；
- disarmed 状态收到 completed trajectory 时清除 command/hold/warmup 状态；
- PX4 从 armed 转为 disarmed 时主动清除历史 trajectory 状态并要求下一条新命令重新建立状态；
- small 验证器起飞前新增 trajectory setpoint 静默窗口、preflight 和 frame error 检查；
- small 验证器在 ARM 后的 helper timeout、MAVLink/Loiter handover 失败、退出 Offboard 失败及正常结束路径统一执行 LAND 并等待 disarm；
- 新增 `--inject-helper-timeout-after-arm`，用于故意制造 ARM 后失败并验收自动收尾；
- avoidance 起飞前状态检查补齐 `pre_flight_checks_pass`。

先执行安全收尾验收（预期验证器以测试失败码退出，但日志必须包含 `SAFETY_CLEANUP` 和 `LAND_COMPLETE`）：

```bash
CONFIRM_SIMULATION_FLIGHT=YES \
PROFILE=small REPEAT_COUNT=1 INJECT_HELPER_TIMEOUT=true \
bash scripts/run_ego_sim_regression.sh
```

通过后再执行正常 0.6 m 三连。不要跳过实际 SITL 验收，也不要仅凭静态检查将 Step 1/Step 2 标记为通过。

### P1：绕障末段仍可能出现规划告警

历史绕障测试中曾出现：

```text
ERROR! the drone is in obstacle. This should not happen.
First 3 control points in obstacles!
Ran out of pool
```

当前自动日志扫描将这些信息视为回归失败。进入 avoidance 严格回归后，必须优先消除或定位这些告警，不能只根据物理轨迹未碰撞就判定通过。

### P1：代码基线尚未完全固化

Ego-Planner、FAST-LIO 和 Livox 驱动的部分关键修改仍在嵌套仓库工作区中。完成仿真严格回归后，需要分别审查并提交，避免依赖旧的 `build/install` 产物。

## 5. 后续执行顺序

后续严格按照以下顺序执行。前一步未通过，不进入下一步。

### Step 1：修复测试隔离和失败收尾

只修改验证流程，不调整飞行性能参数：

1. small/avoidance/takeover 开始前确认不存在旧 trajectory setpoint。
2. 必要时重启 Ego-Planner 和 trajectory interface。
3. 将 small 脚本所有 ARM 后失败路径统一接入 LAND/disarm。
4. 测试开始前要求：
   - PX4 disarmed；
   - 无 failsafe；
   - 接近地面；
   - nav state 为允许的 Position/Loiter 状态；
   - preflight checks 通过；
   - planner/PX4 frame error 在阈值内。

通过标准：故意制造 helper 超时后，PX4 仍能自动 LAND 并 disarm，不遗留 Offboard 控制。

### Step 2：完成 0.6 m small 三连

使用当前保守参数，每轮使用干净规划状态：

```bash
CONFIRM_SIMULATION_FLIGHT=YES \
PROFILE=small \
REPEAT_COUNT=3 \
bash scripts/run_ego_sim_regression.sh
```

每轮必须满足：

- `SMALL_GOAL_RESULT passed=True`
- `bad=none`
- `offboard_seen=True`
- `failsafe=False`
- `LAND_COMPLETE`
- 最终 disarmed
- 危险 planner 日志计数为 0

### Step 3：扩大无障碍短目标

small 0.6 m 三连通过后，依次验证：

1. 1.0 m，三次；
2. 1.5 m，三次。

保持 `max_vel<=0.25`、`max_acc<=0.35`，不同时修改多个参数。

### Step 4：低速 avoidance 严格回归

```bash
CONFIRM_SIMULATION_FLIGHT=YES \
PROFILE=avoidance \
REPEAT_COUNT=3 \
bash scripts/run_ego_sim_regression.sh
```

通过标准：

- 目标直线路径确实穿过障碍物；
- `PASS_CLEARANCE_TEST True`；
- 不进入障碍 AABB；
- 不触碰障碍 XY 区域；
- 满足最小安全距离和目标区净空；
- 无 failsafe；
- LAND/disarm 完成；
- 危险 planner 日志计数为 0。

如果出现目标附近占据告警，优先检查 self-filter、地面噪声、inflation 和目标收敛逻辑，不先提高速度或扩大 A* pool。

### Step 5：外部接管回归

依次验证 active trajectory 中：

1. LAND；
2. RTL；
3. disarm。

要求 trajectory interface 立即停止抢占，PX4 正确退出 Offboard，且无 failsafe。

### Step 6：故障注入

正常回归通过后，再验证：

- LiDAR 中断；
- FAST-LIO odom 中断；
- position command 中断；
- planner/PX4 frame mismatch；
- MicroXRCEAgent 中断。

预期行为是禁止进入 Offboard或退出自主控制，而不是继续盲飞。

### Step 7：固化代码基线

1. 审查 Ego-Planner、FAST-LIO、Livox 嵌套仓库修改。
2. 将通用源码修复与设备 IP、RViz、本机配置分开提交。
3. 从干净工作区重新构建。
4. 重新运行 chain、small、avoidance 和 takeover 回归。

## 6. 当前常用验证工具

### 不解锁 chain 回归

```bash
PROFILE=chain REPEAT_COUNT=3 bash scripts/run_ego_sim_regression.sh
```

### 危险日志扫描

```bash
python3 scripts/check_ego_planner_log.py \
  .runtime/logs/ego_planner_offboard.log
```

扫描以下问题：

- `drone is in obstacle`
- `First 3 control points in obstacles`
- `Ran out of pool`
- 从 `[0,0,0]` 修正局部目标

### 停止并清理仿真

```bash
./scripts/stop_px4_sim.sh
```

## 7. 当前阶段完成标准

在不接入真机的前提下，项目达到以下条件后可认为仿真阶段较为完善：

- chain 严格回归连续通过；
- 0.6/1.0/1.5 m 小目标各连续三次通过；
- avoidance 连续三次通过且危险日志为 0；
- LAND/RTL/disarm 接管全部通过；
- 传感器和通信故障时能安全退出自主控制；
- 每轮测试均可自动收尾到 disarmed；
- 代码和参数已提交，可从干净工作区复现。

达到以上标准前，不进入真机自由绕障测试。