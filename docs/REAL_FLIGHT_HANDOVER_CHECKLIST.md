# 真机接入前交接清单

本文用于冻结仿真成果，并规定后续接入真机时的最低准入条件。**完成本清单不等于允许自由飞行**；首次真机测试仍必须从拆桨台架开始。

## 0. 当前优化结论（2026-07-21）

当前仓库已达到**拆桨固定台架可部署**状态：已有独立真机入口、真实传感器链路配置约束、自动 readiness gate、Offboard 停流与人工接管保护。尚未获得真实飞控、MID360、机体外参、PX4 failsafe 参数和现场测试数据，因此不能把“软件可部署”表述为“已批准带桨飞行”。

本轮完成内容：

- 新增 `scripts/start_real_autonomy.sh`，真机链路固定 `use_sim_time=false`、关闭全部 sim bridge，不启动 Gazebo/PX4 SITL。
- 真机入口要求 `CONFIRM_REAL_BENCH=YES`，且规划链保持 `trajectory_auto_arm=false`，不会自动解锁。
- 拒绝文件名含 `template` 的 Livox/FAST-LIO 配置；要求 `AIRFRAME_CALIBRATION_ID`，并输出两份配置的 SHA-256 短哈希供试验记录追溯。
- 校验 Livox JSON 结构、主机/雷达 IP 合法性，并确认点云接收 IP 已配置在伴随计算机网卡上。
- readiness gate 统一监听真实链路 `/autonomy/local_map`，检查 PX4 preflight/failsafe/disarmed/local-position-valid、消息频率、NaN/Inf、时间戳倒退或停滞、FAST-LIO 静止漂移/跳变、PX4 与 planner 坐标误差、地图非空与 frame 一致性。
- readiness gate 任一项失败即退出并终止所启动的 ROS 进程；启动过程中任一关键 launch 退出也会整体清理。
- 已通过 Bash/Python/launch 静态检查和真机确认门禁负向测试；`px4_trajectory_interface`、`px4_fastlio_bridge` 在 15 秒限制内构建成功（2 packages，0.87 s）。

所有本轮终端查询均通过 `timeout 15s` 执行。后续人工执行耗时命令也应沿用：

```bash
timeout 15s <查询命令>
```

## 1. 已具备的软件安全边界

- 规划链默认 `trajectory_auto_arm=false`，不能自行解锁。
- 仅在 PX4 已解锁、local position 有效且 planner/PX4 坐标对齐时请求 Offboard。
- planner command 中出现 NaN/Inf 时立即拒绝并清空命令状态。
- PX4 vehicle status 或 local position 超过 0.5 s 未更新时停止 Offboard setpoint 流。
- PX4 local position 失效或 PX4 报告 failsafe 时停止规划输出。
- PX4 离开 Offboard，或收到 LAND/RTL/disarm/非 Offboard mode 命令后，规划输出在本次解锁周期内保持抑制，不自动抢回控制权。
- disarm 后清除 trajectory、completed hold、frame offset 和接管抑制状态。

> PX4 必须另行配置 Offboard-loss 行为。伴随计算机停止 setpoint 只是触发 PX4 failsafe 的条件，最终 Hold/Land/RTL 行为由 PX4 参数决定。

## 2. 仿真冻结门槛

以下测试必须使用同一待冻结 commit，从干净构建目录执行；每项均连续通过 3 次：

```bash
PROFILE=chain REPEAT_COUNT=3 bash scripts/run_ego_sim_regression.sh

CONFIRM_SIMULATION_FLIGHT=YES PROFILE=small REPEAT_COUNT=3 \
  bash scripts/run_ego_sim_regression.sh

CONFIRM_SIMULATION_FLIGHT=YES PROFILE=avoidance REPEAT_COUNT=3 \
  bash scripts/run_ego_sim_regression.sh

CONFIRM_SIMULATION_FLIGHT=YES PROFILE=takeover TAKEOVER_COMMAND=all REPEAT_COUNT=3 \
  bash scripts/run_ego_sim_regression.sh
```

另需通过 ARM 后故意失败的自动降落验收：

```bash
CONFIRM_SIMULATION_FLIGHT=YES PROFILE=small REPEAT_COUNT=1 \
  INJECT_HELPER_TIMEOUT=true bash scripts/run_ego_sim_regression.sh
```

验收日志必须满足：无 failsafe、无危险 planner 日志、最终 disarmed、LAND/RTL/disarm 均不被 trajectory interface 抢回。

## 3. 真机到场后必须提供/确认

- 飞控型号、PX4 固件版本、`px4_msgs` 消息版本一致性。
- 伴随计算机型号、CPU/内存/温度、ROS 2 Humble 和 Cyclone/Fast DDS 配置。
- MID360 序列号、IP、点云与 IMU 频率；飞控与伴随计算机时间同步方案。
- LiDAR→IMU、LiDAR→机体 FRD 外参及标定报告；不得直接使用 template 参数。
- FAST-LIO 静止漂移、动态闭环误差、最大延迟和 CPU 峰值记录。
- FAST-LIO 向 PX4 EKF2 的外部视觉/里程计注入方案，以及坐标、时间戳、协方差和创新检查结果。
- 遥控器人工模式、模式开关、kill switch；QGC 遥测链路。
- PX4 Offboard loss、RC loss、data-link loss、定位失效、低电量、地理围栏、最大高度/速度和 RTL 高度参数导出。

## 4. 拆桨真机启动方式

先从模板复制出机体专用配置，写入真实 IP 和已标定外参；禁止直接改名后沿用模板数值。启动前由操作员移除桨叶、固定机体、连接遥控器/QGC，并单独启动与飞控匹配的 Micro XRCE-DDS Agent。

```bash
CONFIRM_REAL_BENCH=YES \
AIRFRAME_CALIBRATION_ID=uav01-mid360-20260721 \
LIVOX_CONFIG=/absolute/path/uav01_mid360.json \
FASTLIO_CONFIG_PATH=/absolute/path/fastlio_config \
FASTLIO_CONFIG_FILE=uav01_mid360.yaml \
bash scripts/start_real_autonomy.sh
```

只有终端出现以下结果才表示软件数据链路通过：

```text
BENCH_READINESS_RESULT passed=True
REAL_AUTONOMY_READY: readiness gate passed
```

该结果仍不替代遥控模式开关、kill switch、PX4 failsafe 参数和螺旋桨方向等人工检查。`Ctrl-C` 会终止两个 launch 并清理子进程。

## 5. 放飞 Go/No-Go 门槛

以下证据全部归档后，才可从拆桨台架进入带桨系留阶段：

- [ ] 本文第 2 节全部仿真回归在同一 commit 连续通过 3 次，日志无危险 planner 事件。
- [ ] Livox 与 FAST-LIO 配置哈希、`AIRFRAME_CALIBRATION_ID`、标定报告三者一致。
- [ ] `BENCH_READINESS_RESULT passed=True` 连续 3 次，每次至少采样 8 秒。
- [ ] 飞控固件与 `px4_msgs` 版本匹配，DDS Agent 重连测试通过。
- [ ] PX4 参数导出已审查：Offboard loss、RC loss、data-link loss、定位失效、低电量、geofence、RTL。
- [ ] 拔除 LiDAR、停止 DDS Agent、停止规划器三项故障注入均触发预期停流/PX4 failsafe，且不自动抢回 Offboard。
- [ ] QGC LAND、RTL、遥控模式切换和 kill switch 在拆桨台架逐项通过。
- [ ] CPU/内存/温度和端到端延迟在 10 分钟运行中无持续恶化，FAST-LIO 无不可接受漂移或跳变。
- [ ] 现场安全负责人批准测试区域、系留设施、人员站位和中止条件。

任一项未勾选均为 **No-Go**。本仓库无法替代硬件现场完成这些勾选。

## 6. 真机分级测试顺序

1. **拆桨固定台架**：运行 `check_real_bench_readiness.py`，人工切换 Position/Offboard/Land/RTL，验证 kill switch。
2. **带桨系留/保护网**：人工起降与定点，不给自主横向目标。
3. **0.3–0.6 m 短平移**：低高度、低速、无障碍，操作员全程握持模式开关。
4. **静态大障碍**：仅一个软质/可破坏障碍，扩大安全距离和目标区净空。
5. **多障碍与扩大包线**：只有前一级连续通过后才允许增加速度、距离或复杂度，每次只改一个变量。

任一级发生定位跳变、时间倒退、点云空洞、危险规划日志、控制抢占、通信超时或 PX4 failsafe，立即停止升级并回到拆桨台架定位原因。
