# PX4 + MID360 + FAST-LIO + Ego-Planner 真机部署 SOP

本文是现场操作流程。执行人应逐项勾选并保存证据。软件状态和剩余放飞门槛见 `docs/REAL_FLIGHT_HANDOVER_CHECKLIST.md`。

> 安全边界：首次部署必须拆除全部螺旋桨并固定机体。本文中“门禁通过”仅表示可继续台架测试，不表示允许带桨或自由飞行。任一步出现异常，停止升级测试，不得绕过检查器。

## 1. 适用范围与职责

适用链路：

```text
PX4 -> Micro XRCE-DDS Agent -> ROS 2
MID360 -> livox_ros_driver2 -> FAST-LIO -> /Odometry
FAST-LIO bridge -> /odom + /autonomy/cloud_registered
Ego-Planner -> trajectory_interface -> PX4 Offboard
```

现场至少两人：

- 操作员：控制遥控器模式开关、LAND/RTL 和 kill switch。
- 监控员：操作伴随计算机、QGC，记录日志并宣读 Go/No-Go 项。

任何人均可发出“停止”；发出后立即退出 Offboard，必要时 LAND/kill，不讨论后再执行。

## 2. 必备输入

部署前填写：

| 项目 | 现场值 |
|---|---|
| 日期/地点 | |
| Git commit | |
| PX4 飞控/固件版本 | |
| `px4_msgs` 对应版本 | |
| 伴随计算机/ROS 2 版本 | |
| DDS 连接方式（UDP/串口） | |
| DDS 串口或 UDP 端口 | |
| MID360 序列号/IP | |
| 伴随计算机 Livox 网卡/IP | |
| Livox 配置绝对路径 | |
| FAST-LIO 配置目录/文件 | |
| `AIRFRAME_CALIBRATION_ID` | |
| 遥控操作员 | |
| 现场安全负责人 | |

必须具备：

- [ ] 机体专用 Livox JSON，不使用 `*template*` 文件。
- [ ] 机体专用 FAST-LIO YAML，包含已验证的 LiDAR-IMU 外参。
- [ ] 外参标定报告与唯一 `AIRFRAME_CALIBRATION_ID`。
- [ ] PX4 参数导出文件和参数审查记录。
- [ ] 遥控器、QGC、急停方案、系留设施和隔离区域。

## 3. 命令执行规则

所有只读查询、语法检查和短诊断必须在 15 秒后自动终止：

```bash
timeout 15s <查询命令>
```

常驻进程和规定采样时间超过 15 秒的测试不能直接套用该限制，包括 DDS Agent、`start_real_autonomy.sh`、10 分钟稳定性测试。它们必须在独立终端运行，并由操作员监控和人工终止。脚本内部 readiness 查询已自带 `timeout 15s`。

严禁使用 `sudo` 启动 ROS 节点。串口/网卡权限应在部署前配置完成。

## 4. 部署前冻结与构建

### 4.1 冻结代码

```bash
cd /home/p/px4_ros2_ws
timeout 15s git status --short
timeout 15s git rev-parse HEAD
timeout 15s git submodule status --recursive
```

- [ ] 记录主仓库 commit。
- [ ] 记录嵌套仓库/submodule commit 和工作区修改。
- [ ] 本次部署使用的源码与仿真回归源码一致。
- [ ] 不依赖来源不明的旧 `build/`、`install/` 产物。

### 4.2 构建

完整首次构建可能超过 15 秒，作为构建操作在受控终端执行：

```bash
cd /home/p/px4_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

构建后执行短查询：

```bash
timeout 15s bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/p/px4_ros2_ws/install/setup.bash
  ros2 pkg executables px4_fastlio_bridge
  ros2 pkg executables px4_trajectory_interface
'
```

- [ ] 构建返回码为 0。
- [ ] 能找到 `lio_odometry_bridge`、`trajectory_interface` 等可执行程序。
- [ ] 构建日志无被忽略的 ABI、消息版本或依赖错误。

## 5. 飞控与 PX4 配置

### 5.1 机械和供电

- [ ] 全部螺旋桨已拆除，电机轴无附着物。
- [ ] 机体固定，测试时不会因电机动作移动或翻转。
- [ ] 飞控、伴随计算机、MID360 共地和供电容量满足峰值需求。
- [ ] USB/串口/网线有应力释放，不会被振动拔出。
- [ ] 遥控器已连接，模式开关和 kill switch 可立即触达。

### 5.2 参数审查

在 QGC 中导出并由两人复核：

- [ ] Offboard setpoint 丢失行为和超时时间。
- [ ] RC 丢失、data-link 丢失和定位失效行为。
- [ ] 低电量动作、geofence、高度/速度限制。
- [ ] RTL 高度和 RTL/LAND 路径适合现场环境。
- [ ] EKF2 使用的定位源、坐标系、延迟和协方差配置正确。
- [ ] PX4 uXRCE-DDS client 的传输方式、端口/波特率与 Agent 一致。

此项目的 trajectory interface 在状态/位置超时或 PX4 failsafe 时停止 setpoint；最终 Hold/Land/RTL 动作由 PX4 参数决定，不能省略本节。

## 6. MID360 网络与标定配置

从模板创建机体专用文件，禁止直接使用模板：

```bash
cp src/livox_ros_driver2/config/MID360_config.mid360_nuc_template.json \
  /absolute/path/uav01_mid360.json
cp src/px4_fastlio_bridge/config/mid360_ego_planner.template.yaml \
  /absolute/path/fastlio_config/uav01_mid360.yaml
```

编辑并复核：

- Livox JSON 中 `host_net_info` 的四个 IP 均为伴随计算机专用网卡 IP。
- `lidar_configs[0].ip` 为当前 MID360 IP。
- FAST-LIO `common.lid_topic=/livox/lidar`、`common.imu_topic=/livox/imu`。
- `mapping.extrinsic_T`、`mapping.extrinsic_R` 来自当前机体标定报告。
- 时间单位、扫描线、盲区、量程和频率与 MID360 驱动输出一致。

网络短查询：

```bash
timeout 15s ip -br -4 address
timeout 15s ping -c 3 -W 1 <MID360_IP>
timeout 15s sha256sum /absolute/path/uav01_mid360.json \
  /absolute/path/fastlio_config/uav01_mid360.yaml
```

- [ ] 本机 Livox IP 已绑定到物理网卡。
- [ ] MID360 可达且无重复 IP。
- [ ] 两份配置哈希已记录，与批准的标定记录一致。

## 7. 启动顺序

准备三个终端。任何终端异常退出均执行第 11 节收尾，不继续任务。

### 7.1 终端 A：启动 DDS Agent

UDP 示例：

```bash
MicroXRCEAgent udp4 -p 8888
```

串口示例，仅在 PX4 配置与设备名确认后使用：

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

不要同时启动两个 Agent 连接同一飞控。确认日志出现 client 建立，再做短查询：

```bash
timeout 15s bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/p/px4_ros2_ws/install/setup.bash
  ros2 topic list | grep /fmu/
'
```

- [ ] `/fmu/out/vehicle_status_v4` 存在。
- [ ] `/fmu/out/vehicle_local_position_v1` 存在。
- [ ] `/fmu/out/sensor_combined` 存在。
- [ ] QGC 显示飞控在线且当前为 disarmed。

### 7.2 终端 B：启动真机自主链路

保持拆桨、固定和 disarmed：

```bash
cd /home/p/px4_ros2_ws

CONFIRM_REAL_BENCH=YES \
AIRFRAME_CALIBRATION_ID=uav01-mid360-20260721 \
LIVOX_CONFIG=/absolute/path/uav01_mid360.json \
FASTLIO_CONFIG_PATH=/absolute/path/fastlio_config \
FASTLIO_CONFIG_FILE=uav01_mid360.yaml \
LAUNCH_RVIZ=false \
bash scripts/start_real_autonomy.sh 2>&1 | tee \
  "real_bench_$(date +%Y%m%d_%H%M%S).log"
```

脚本会：

1. 拒绝缺失确认、缺失标定 ID 和模板配置。
2. 校验 Livox JSON、IP 和本机网卡。
3. 启动真实 Livox/FAST-LIO 链路与规划/控制链路。
4. 强制关闭 sim time、sim bridge 和自动解锁。
5. 自动运行最多 15 秒的 readiness gate。
6. 任一门禁失败时退出并清理 ROS 子进程。

### 7.3 启动通过标准

必须同时看到：

```text
LIVOX_CONFIG_OK ... sha256=...
FASTLIO_CONFIG_OK calibration=... sha256=...
BENCH_READINESS_RESULT passed=True
REAL_AUTONOMY_READY: readiness gate passed
```

- [ ] PX4 为 disarmed、preflight checks passed、failsafe false。
- [ ] LiDAR/IMU/里程计/地图消息频率全部达标。
- [ ] 无 NaN/Inf、时间戳倒退或超限停滞。
- [ ] FAST-LIO 静止漂移和单步跳变低于门限。
- [ ] planner 与 PX4 水平/垂直坐标误差低于门限。
- [ ] `/autonomy/local_map` 非空且 frame 为 `world`。

任一 `FAIL:`、进程退出、定位跳变或危险 planner 日志均为 No-Go。

## 8. 拆桨台架验收

保持机体固定，按顺序执行，每项保存 QGC 视频和 ROS 日志：

1. 静置运行 10 分钟，记录 CPU、内存、温度和 FAST-LIO 漂移。
2. 缓慢改变机体姿态，确认点云、里程计方向与实际动作一致。
3. 遥控切换 Position/人工模式，确认规划链不抢占。
4. 测试 QGC LAND、RTL 和遥控模式切换。
5. 在拆桨条件下验证 kill switch。
6. 连续重启自主链路 3 次，每次 readiness 必须通过。

短状态查询示例：

```bash
timeout 15s bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/p/px4_ros2_ws/install/setup.bash
  ros2 topic hz /fmu/out/vehicle_status_v4 --window 20
'
```

注意：`ros2 topic hz` 本身常驻，必须保留外层 `timeout 15s`。

## 9. 故障注入验收

仅在拆桨、固定机体和操作员握持 kill switch 时执行：

| 故障 | 预期软件行为 | 预期 PX4 行为 |
|---|---|---|
| 停止规划器 | command 超时，不继续新轨迹 | 按已审查策略保持/退出 Offboard |
| 停止 DDS Agent | PX4 状态/位置超时，trajectory interface 停流 | 触发 Offboard-loss 动作 |
| 断开 MID360 | LiDAR/地图率归零，readiness 不通过 | 不允许开始新自主任务 |
| FAST-LIO 跳变/失效 | 坐标或稳定性门禁失败 | 定位失效策略按 PX4 参数执行 |
| QGC LAND/RTL | 输出立即抑制，本次 armed 周期不抢回 | 执行 LAND/RTL |
| 遥控切出 Offboard | 输出抑制 | 人工模式保持控制权 |

每项必须确认：

- [ ] trajectory interface 没有自动抢回 Offboard。
- [ ] PX4 实际动作与参数审查记录一致。
- [ ] 恢复数据源后不会在同一 armed 周期自动恢复危险轨迹。
- [ ] 故障、时间、日志和恢复步骤已归档。

## 10. 带桨测试升级

只有 `docs/REAL_FLIGHT_HANDOVER_CHECKLIST.md` 的 Go/No-Go 项全部勾选并由安全负责人签字后执行：

1. 带桨系留/保护网，仅人工起降和定点。
2. 低高度 0.3 m 横移，无障碍，速度/加速度保持默认 `0.25 m/s`、`0.35 m/s²` 以下。
3. 0.6 m 横移，连续通过 3 次。
4. 单个软质大障碍，保持足够净空。
5. 多障碍和扩大包线，每次只改变一个变量。

每次起飞前重复：

- [ ] 电池、桨叶、旋向、电机、重心、紧固件检查完成。
- [ ] QGC/PX4 无告警，GPS/外部视觉状态符合当前方案。
- [ ] readiness 连续通过，当前日志已开始记录。
- [ ] LAND、RTL、模式开关、kill switch 操作员口令确认。
- [ ] 人员撤离，隔离区清空，风和环境条件符合限制。

立即中止条件：定位跳变、点云空洞、时间倒退、规划器障碍告警、轨迹突变、通信超时、CPU 热降频、PX4 failsafe、人工接管失败或任何人员进入隔离区。

## 11. 正常停止与异常收尾

正常停止：

1. 人工切出 Offboard。
2. LAND 并确认落地。
3. 确认 PX4 disarmed。
4. 在真机自主链路终端按 `Ctrl-C`，确认子进程退出。
5. 停止 DDS Agent。
6. 断开主动力，再断开伴随计算机/MID360 电源。

异常时优先顺序：人工模式或 LAND/RTL -> kill switch（仅按现场预案）-> 人员避让 -> 断电。不得为了保存日志延迟安全动作。

退出后短查询：

```bash
timeout 15s pgrep -af 'trajectory_interface|fastlio_mapping|livox_ros_driver2_node'
timeout 15s git status --short
```

第一条无输出才表示相关进程已退出；若仍有进程，先确认飞控 disarmed，再结束残留进程。

## 12. 证据归档

每次测试保存到独立目录：

```text
flight_records/YYYYMMDD_HHMMSS/
  operator.md
  git_commit.txt
  git_status.txt
  px4_params.params
  calibration_report.pdf
  config_sha256.txt
  real_bench.log
  qgc.ulg
  rosbag2/
  result.md
```

`result.md` 至少记录：测试级别、目标、环境、配置哈希、PX4 参数版本、readiness 结果、异常、人工接管情况、最终 armed 状态和 Go/No-Go 结论。

## 13. 最终放行签字

| 角色 | 姓名 | 结论 | 日期/签字 |
|---|---|---|---|
| 软件负责人 | | Go / No-Go | |
| 飞控负责人 | | Go / No-Go | |
| 遥控操作员 | | Go / No-Go | |
| 现场安全负责人 | | Go / No-Go | |

任一角色为 No-Go 或未签字，禁止升级到下一测试级别。