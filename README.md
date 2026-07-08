# PX4 ROS 2 自主飞行工作区

这个工作区是一个面向 PX4 多旋翼自主飞行的 ROS 2 Humble 基础骨架。它基于 PX4 的 ROS 2 接口库和自定义模式机制，而不是旧式的 ROS Offboard 示例。

如果你想深入理解代码结构、控制循环、任务来源、避障状态机和感知数据流，请看 [代码和实现原理说明](docs/CODE_IMPLEMENTATION_README.md)。

## 工作区结构

- `src/px4_msgs`
  PX4 的 ROS 2 消息定义。这里的版本需要和你的 PX4 固件版本保持一致。
- `src/px4_ros2_interface_lib`
  PX4 ROS 2 控制接口库和自定义模式库。
- `src/px4_autonomy_mode`
  你自己的 ROS 2 自主飞行模式包。后续任务逻辑、感知接入和安全策略都建议在这里扩展。
- `src/px4_autonomy_bringup`
  启动文件集合，用来统一拉起自主模式和可选的 Livox 驱动。
- `src/livox_ros_driver2`
  Livox 的 ROS 2 驱动。

## 安全策略

这个工作区的设计原则是：自主模式不负责解锁。

- 先用遥控器或 QGC 在 PX4 原生模式下解锁、起飞，比如 `Position` 模式。
- 飞机状态稳定后，再切换到 `ROS2 Autonomy`。
- 如果自主飞行出现异常，立刻通过遥控器或 QGC 切回 PX4 原生手动或辅助模式。
- 当 PX4 将自主模式取消激活后，ROS 2 节点就不再是当前控制源。

也就是说，手动接管不是补救措施，而是默认的回退路径。

## 推荐模式策略

- `Manual/Stabilized` 或 `Position`
  作为人工接管的主要模式。
- `ROS2 Autonomy`
  作为飞行中自主执行任务的模式，建议只在飞机状态健康、定位可靠时切入。
- `Land` 或 `RTL`
  作为遥控器或 QGC 触发的应急恢复模式。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/p/px4_ros2_ws
export LD_LIBRARY_PATH=/home/p/px4_ros2_ws/local/livox_sdk2/lib:$LD_LIBRARY_PATH
colcon build
source install/setup.bash
```

## 运行

先启动 PX4 SITL 或真机 PX4，并确保 `Micro XRCE-DDS Agent` 已经启动，然后执行：

```bash
source /opt/ros/humble/setup.bash
source /home/p/px4_ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/p/px4_ros2_ws/local/livox_sdk2/lib:$LD_LIBRARY_PATH
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py
```

如果需要同时启动 Livox 驱动：

```bash
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py use_livox:=true
```

也可以在启动时直接覆盖任务参数，例如。室内障碍物世界建议先用小尺寸、低速度：

```bash
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py \
  mission_size_m:=2.5 \
  mission_altitude_m:=2.0 \
  hold_time_s:=3.0 \
  acceptance_radius_m:=0.35 \
  max_horizontal_velocity_m_s:=0.8 \
  max_vertical_velocity_m_s:=0.5 \
  max_heading_rate_deg_s:=60.0 \
  mission_timeout_s:=180.0 \
  auto_rtl_after_finish:=false \
  obstacle_distance_topic:=/perception/min_obstacle_distance \
  obstacle_stop_distance_m:=2.0 \
  obstacle_abort_distance_m:=1.0 \
  obstacle_hold_timeout_s:=5.0 \
  enable_obstacle_hold:=true \
  launch_obstacle_sim:=false
```

## SITL 联调流程

### 一键启动脚本

日常仿真推荐直接使用脚本启动，不需要手动打开多个终端：

```bash
/home/p/px4_ros2_ws/scripts/start_px4_sim.sh
```

默认会启动室内障碍物世界 `room_obstacles`。脚本会把 `/home/p/px4_ros2_ws/sim/worlds/*.sdf` 自动同步到 PX4 的 Gazebo world 目录。

脚本会依次启动：

- `PX4 SITL`
- `MicroXRCEAgent`
- `QGroundControl`
- `ROS2 Autonomy`

其中 `ROS2 Autonomy` 会等待 `/fmu/...` 话题出现后再启动，避免 PX4 和 ROS 2 通信还没建立就注册外部模式失败。

停止全部进程：

```bash
/home/p/px4_ros2_ws/scripts/stop_px4_sim.sh
```

如果你想退回 PX4 默认空旷世界：

```bash
PX4_WORLD=default /home/p/px4_ros2_ws/scripts/start_px4_sim.sh
```

同时启动障碍物距离模拟节点：

```bash
LAUNCH_OBSTACLE_SIM=true OBSTACLE_SIM_MODE=hold /home/p/px4_ros2_ws/scripts/start_px4_sim.sh
```

也可以组合使用：

```bash
PX4_WORLD=room_obstacles LAUNCH_OBSTACLE_SIM=true OBSTACLE_SIM_MODE=wave \
  /home/p/px4_ros2_ws/scripts/start_px4_sim.sh
```

下面是一套推荐的本地联调顺序，适合先在仿真里把 `PX4 + ROS 2 + QGC` 跑通。

### 1. 启动 PX4 SITL

如果你本机已经准备好了 `PX4-Autopilot`，可以在 PX4 源码目录下启动 Gazebo 仿真，例如：

```bash
cd ~/PX4-Autopilot
make px4_sitl_default gz_x500
```

如果你使用的不是 `x500` 机型，可以替换成你自己的 SITL 目标。

### 2. 启动 Micro XRCE-DDS Agent

PX4 和 ROS 2 之间的通信需要这个代理进程：

```bash
MicroXRCEAgent udp4 -p 8888
```

如果 PX4 侧端口配置不是 `8888`，这里需要保持一致。

### 3. 启动 QGroundControl

进入 `QGroundControl.AppImage` 所在目录后运行：

```bash
./QGroundControl.AppImage
```

建议使用较新的 Daily Build，这样对 ROS 2 自定义模式的兼容性更好。

### 4. 启动 ROS 2 工作区

```bash
source /opt/ros/humble/setup.bash
source /home/p/px4_ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/p/px4_ros2_ws/local/livox_sdk2/lib:$LD_LIBRARY_PATH
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py
```

如果你还想一起拉起 Livox：

```bash
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py use_livox:=true
```

如果你要在 Gazebo 里测试 Ego-Planner 绕飞，建议优先使用原生 `PointCloud2`，不要默认走“扫描线转点云”：

```bash
source /opt/ros/humble/setup.bash
source /home/p/px4_ros2_ws/install/setup.bash
ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py \
  use_native_3d_pointcloud:=true \
  native_pointcloud_topic:=/sim/mid360/points \
  use_scan_fallback:=false
```

如果你想用更接近真机 `Mid360` 挂载方式的 Gazebo Classic 机型，可以直接：

```bash
PX4_MODEL=iris_mid360_sim /home/p/px4_ros2_ws/scripts/start_px4_sim.sh
```

如果你的仿真暂时还没有深度相机或 3D lidar 点云，可以临时退回旧路径：

```bash
ros2 launch px4_trajectory_interface ego_planner_offboard.launch.py \
  use_native_3d_pointcloud:=false \
  use_scan_fallback:=true
```

## 推荐操作顺序

建议按下面顺序操作，尤其是前期调试阶段：

1. 确认 PX4、QGC、Micro XRCE-DDS Agent、ROS 2 节点都已经正常启动。
2. 在 QGC 中确认飞机状态健康、定位状态正常、RC 输入正常。
3. 先使用 `Position` 或其他 PX4 原生模式进行解锁和起飞。
4. 飞机稳定悬停后，再切换到 `ROS2 Autonomy`。
5. 如果自主飞行表现异常，立即通过 RC 或 QGC 切回 `Position`、`Manual`、`Land` 或 `RTL`。

## 手动接管说明

这个工作区里，自主模式默认不是唯一控制入口，而是可中断的任务模式。

- 推荐将遥控器保留为最高优先级的人为接管手段。
- 推荐在 RC 上单独映射一个稳定可靠的回退模式，比如 `Position`。
- 推荐把 `Land` 或 `RTL` 作为第二条恢复路径。
- 当前 `px4_autonomy_mode` 默认 `preventArming(true)`，也就是它不会主动负责解锁起飞。

这意味着更安全的做法是：

- 人先起飞
- ROS2 接管任务
- 人随时切回

## 当前自主模式说明

当前 `px4_autonomy_mode` 是一个更像任务控制节点的基础版本，但仍然不是最终任务系统。

- 启动后会注册一个名为 `ROS2 Autonomy` 的用户可选模式。
- 模式内部使用一个简单的方形航线示例。
- 支持配置任务边长、任务高度、停留时间、接受半径、水平速度、垂直速度、航向角速度和任务超时。
- 到达最终点后会短暂停留，然后上报任务完成。
- 飞行途中可以通过 `/autonomy/target_ned` 发布新的本地目标点，临时改变目的地。
- 飞行过程中会在 ROS 2 终端里打印当前目标航点和状态变化。
- 它适合作为后续扩展入口，用来接入航点任务、感知触发、避障逻辑和模式执行器。

### 当前支持的任务参数

- `mission_size_m`
  方形任务的边长，单位米。
- `mission_altitude_m`
  任务目标高度，单位米。
- `hold_time_s`
  到达最终点后的停留时间，单位秒。
- `acceptance_radius_m`
  判定“到达航点”的距离阈值，单位米。
- `max_horizontal_velocity_m_s`
  最大水平速度，单位米每秒。
- `max_vertical_velocity_m_s`
  最大垂直速度，单位米每秒。
- `max_heading_rate_deg_s`
  最大航向角速度，单位度每秒。
- `mission_timeout_s`
  任务超时，超时后模式会直接完成并把控制权交还给 PX4。
- `auto_rtl_after_finish`
  当前用于表达“任务结束后需要由 QGC/遥控器接管返航”的意图；实际是否切到 `RTL` 取决于你启用的上层接管链路和运行时模式配置。
- `obstacle_distance_topic`
  后续感知节点发布最近障碍距离的 ROS 2 话题。
- `obstacle_stop_distance_m`
  最近障碍距离小于该阈值时，自主模式进入原地保持。
- `obstacle_abort_distance_m`
  最近障碍距离小于该阈值时，自主模式立即中止，把控制权交回 PX4。
- `obstacle_hold_timeout_s`
  障碍物保持状态持续超过该时间后，中止自主模式。
- `enable_obstacle_hold`
  是否开启障碍物保护逻辑。
- `target_point_topic`
  飞行途中接收目标点的话题，默认 `/autonomy/target_ned`。
- `accept_runtime_target`
  是否允许飞行途中通过话题修改目的地。

### 飞行中修改目标点

当前推荐先用 ROS 2 话题修改目标点。坐标使用 PX4 本地 NED 坐标，单位米：

- `x`
  向北/仿真局部 X 方向，单位米。
- `y`
  向东/仿真局部 Y 方向，单位米。
- `z`
  NED 高度，向上是负数。比如离地 2 米写 `-2.0`。

例如，让飞机飞向房间内的 `[1.0, 1.0, -2.0]`：

```bash
source /opt/ros/humble/setup.bash
source /home/p/px4_ros2_ws/install/setup.bash
ros2 topic pub --once /autonomy/target_ned geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 1.0, y: 1.0, z: -2.0}}"
```

再发一个新点，飞机会在 `ROS2 Autonomy` 模式中改飞到新目的地：

```bash
ros2 topic pub --once /autonomy/target_ned geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: -1.5, y: 1.2, z: -2.0}}"
```

鼠标点击 Gazebo 画面改目标可以做，但需要额外实现 Gazebo GUI 插件或 RViz/网页交互节点，把点击位置转换成 `/autonomy/target_ned`。QGC 地图点击不适合这个室内房间，因为 QGC 操作的是经纬度地图，不是 Gazebo 房间坐标。

### 激光雷达接入建议

当前版本已经为激光雷达/建图结果预留了一个最简单的感知接口：

- 发布一个 `std_msgs/msg/Float32`
- 话题默认是 `/perception/min_obstacle_distance`
- 含义是“当前最近障碍距离”，单位米

推荐的演进顺序：

1. 先从点云/局部地图节点里算出最近障碍距离
2. 发布到 `/perception/min_obstacle_distance`
3. 先验证 `Hold / Abort`
4. 再逐步扩展成局部重规划或自动绕障

### 仿真测试障碍物响应

为了在没有真实激光雷达的情况下验证 `Hold / Abort`，工作区新增了一个最小的障碍物模拟节点：

- 包名：`px4_obstacle_tools`
- 节点：`obstacle_distance_sim`

它会向默认话题 `/perception/min_obstacle_distance` 发布一个最近障碍距离。

#### 直接通过 bringup 一起启动

```bash
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py \
  launch_obstacle_sim:=true \
  obstacle_sim_mode:=hold
```

可选模式：

- `safe`
  发布安全距离，自主模式正常飞行
- `hold`
  发布接近障碍物的距离，触发 `ObstacleHold`
- `abort`
  发布极近障碍物的距离，直接触发 `Abort`
- `wave`
  在安全/危险距离之间来回变化，适合观察状态切换

#### 单独启动模拟节点

```bash
ros2 launch px4_obstacle_tools obstacle_sim.launch.py mode:=hold
```

## 真机接入前建议

在上真机前，建议至少完成下面这些检查：

1. SITL 下完整跑通一次：
   手动起飞 -> 切 `ROS2 Autonomy` -> 再切回手动模式。
2. 确认 `px4_msgs` 和 PX4 固件版本严格匹配。
3. 确认 RC 模式切换、急停、`RTL`、`Land` 都能稳定工作。
4. 确认定位输入在 ROS 2 和 PX4 两边都稳定，没有跳变。
5. 在允许自主接管前，增加定位有效性和任务前置条件检查。

## 后续扩展建议

- 将 `px4_autonomy_mode` 里的方形航线骨架替换成你自己的航点、走廊跟踪或任务执行逻辑。
- 在允许切入自主模式前，增加定位输入有效性检查。
- 增加感知触发的中止逻辑，在必要时自动执行 `Land`、`RTL`，或者切回 `Position` 模式。
