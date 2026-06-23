# 代码和实现原理说明

这份文档用于理解当前 PX4 ROS 2 自主飞行工作区的代码结构、运行链路和核心实现。根目录 `README.md` 更适合作为项目主页和快速上手；这里更偏向“我要改代码前，先搞懂它怎么工作”。

## 一句话架构

这个工程把 PX4 当作底层飞控和安全边界，把 ROS 2 当作任务决策层：

```text
QGC/遥控器
  -> PX4 Autopilot
  -> Micro XRCE-DDS Agent
  -> /fmu/... ROS 2 topics
  -> px4_ros2_interface_lib
  -> px4_autonomy_mode
  -> GotoSetpoint 目标点控制
```

感知链路则独立把 Gazebo/Livox/点云转换成简单距离输入：

```text
Gazebo scan / PointCloud2 / simulated distance
  -> px4_obstacle_tools
  -> /perception/... Float32 distance topics
  -> px4_autonomy_mode obstacle logic
```

最重要的设计取舍是：自主模式不负责解锁，不负责替代 PX4 的底层安全机制。它只在用户切到 `ROS2 Autonomy` 后，持续给 PX4 发布目标点。

## 主要目录

- `src/px4_autonomy_mode`
  自主飞行模式核心包。当前大部分业务逻辑都在这里。
- `src/px4_autonomy_mode/include/px4_autonomy_mode/autonomy_mode.hpp`
  核心类 `Px4AutonomyMode`，包含参数、订阅、任务、避障、状态机和目标点更新。
- `src/px4_autonomy_mode/src/main.cpp`
  程序入口，创建 `NodeWithMode<Px4AutonomyMode>`。
- `src/px4_autonomy_bringup/launch/autonomy_stack.launch.py`
  统一启动文件，把自主模式、Livox、Gazebo/点云距离工具组合起来。
- `src/px4_obstacle_tools`
  感知辅助工具包，把点云、Gazebo scan 或模拟数据转换成距离话题。
- `scripts/start_px4_sim.sh`
  一键仿真脚本，负责拉起 PX4、Gazebo、Agent、QGC 和 ROS 2。
- `missions`
  多航点任务文件。
- `tools/waypoint_editor.html`
  浏览器航点编辑器。

## 自主模式入口

入口文件很短：

```cpp
// src/px4_autonomy_mode/src/main.cpp
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<Px4AutonomyMode>>(
  "px4_autonomy_mode", true));
rclcpp::shutdown();
```

这里的关键是 `px4_ros2::NodeWithMode<Px4AutonomyMode>`。它来自 `px4_ros2_interface_lib`，作用是：

1. 创建普通 ROS 2 node。
2. 实例化你的模式类 `Px4AutonomyMode`。
3. 把这个模式注册给 PX4 外部模式系统。
4. 在模式激活后周期调用 `updateSetpoint()`。

`Px4AutonomyMode` 的构造函数里定义了 QGC/PX4 里看到的模式名称：

```cpp
ModeBase(node, Settings{"ROS2 Autonomy"}.preventArming(true))
```

`preventArming(true)` 很重要：这个模式不允许自己负责解锁。推荐流程是先用 `Position` 等 PX4 原生模式解锁起飞，稳定悬停后再切入 `ROS2 Autonomy`。

## 核心类 Px4AutonomyMode

核心类定义在：

```text
src/px4_autonomy_mode/include/px4_autonomy_mode/autonomy_mode.hpp
```

它继承：

```cpp
class Px4AutonomyMode : public px4_ros2::ModeBase
```

最重要的成员有两类。

PX4 ROS 2 接口：

```cpp
std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
```

- `_goto_setpoint` 用来给 PX4 发送多旋翼目标位置、朝向、速度上限。
- `_vehicle_local_position` 用来读取当前位置、速度、航向。

ROS 2 订阅：

```cpp
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _obstacle_distance_sub;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _front_obstacle_distance_sub;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _left_obstacle_distance_sub;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _right_obstacle_distance_sub;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _up_obstacle_distance_sub;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _down_obstacle_distance_sub;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _target_point_sub;
rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr _waypoints_sub;
```

这些订阅提供两类输入：

- `/perception/...` 障碍物距离
- `/autonomy/...` 运行时目标点或航点任务

## 生命周期

`Px4AutonomyMode` 主要靠三个回调工作。

### onActivate()

当用户在 QGC/PX4 中切到 `ROS2 Autonomy` 时调用。

它做的事情：

1. 读取当前 NED 位置作为任务起点。
2. 清空保持、完成、绕行、超时相关状态。
3. 记录任务开始时间。
4. 调用 `buildMission()` 生成本次任务航点。
5. 打印任务参数和话题信息。

这意味着默认方形任务不是以仿真世界原点为起点，而是以切入模式那一刻的当前位置为起点。

### updateSetpoint(float dt_s)

这是核心控制循环。模式激活期间，PX4 ROS 2 interface lib 会周期性调用它。

执行顺序大致是：

```text
如果没有航点 -> 失败结束
检查任务总超时
处理障碍物逻辑
取当前航点
计算期望航向
发布 goto setpoint
判断是否到达当前航点
推进航点或进入最终保持
任务完成后 completed(Success)
```

代码里障碍物逻辑放在正常航点推进之前：

```cpp
if (handleObstacleLogic()) {
  return;
}
```

这表示一旦避障/保持接管，本轮不会继续朝任务航点飞。

### onDeactivate()

当用户切出 `ROS2 Autonomy`，或者 PX4 取消这个外部模式时调用。

当前实现只打印提示：

```cpp
RCLCPP_WARN(node().get_logger(),
  "Autonomy mode deactivated. Vehicle control returns to the newly selected PX4 mode.");
```

真正的控制权切换由 PX4 完成。

## 坐标系

任务目标使用 PX4 本地 NED 坐标：

- `x`: North，向北为正。
- `y`: East，向东为正。
- `z`: Down，向下为正。

所以“飞到离地 2 米高度”通常写作：

```text
z = -2.0
```

默认方形任务会把参数 `mission_altitude_m` 转成负 z：

```cpp
const float z_target = -static_cast<float>(std::fabs(_mission_altitude_m));
```

但任务文件里的 `z` 会被直接使用，不会自动转负。写任务文件时建议显式写 `z: -2.0` 这样的 NED 值。

## 参数系统

构造函数里先声明参数：

```cpp
node.declare_parameter("mission_size_m", 8.0);
node.declare_parameter("mission_altitude_m", 4.0);
node.declare_parameter("hold_time_s", 2.0);
...
```

然后读取到成员变量：

```cpp
node.get_parameter("mission_size_m", _mission_size_m);
node.get_parameter("mission_altitude_m", _mission_altitude_m);
...
```

这些参数主要从 `autonomy_stack.launch.py` 传入。`scripts/start_px4_sim.sh` 又会把环境变量转换成 launch 参数。例如：

```bash
MISSION_SIZE_M=2.5 ./scripts/start_px4_sim.sh
```

最终会进入：

```bash
mission_size_m:=2.5
```

## 任务来源优先级

`buildMission()` 会生成 `_mission_waypoints`。优先级是：

1. 运行时收到的多航点任务 `/autonomy/waypoints_ned`
2. 运行时收到的单目标点 `/autonomy/target_ned`
3. 启动参数 `mission_file`
4. 启动参数 `mission_waypoints_ned`
5. 默认方形航线

对应代码逻辑是：

```cpp
if (_manual_mission_received && !_manual_mission_waypoints_ned.empty()) {
  _mission_waypoints = _manual_mission_waypoints_ned;
  return;
}

if (_manual_target_received && _manual_target_ned_m.has_value()) {
  _mission_waypoints.push_back(*_manual_target_ned_m);
  return;
}

if (!_configured_mission_waypoints_ned.empty()) {
  _mission_waypoints = _configured_mission_waypoints_ned;
  return;
}

// fallback: square mission
```

默认方形航线会生成 5 个点：

```text
(x0, y0, z)
(x0 + size, y0, z)
(x0 + size, y0 + size, z)
(x0, y0 + size, z)
(x0, y0, z)
```

`x0/y0` 是切入模式那一刻的位置。

## 航点推进逻辑

当前航点索引用 `_waypoint_index` 保存。

每次控制循环：

1. 取当前目标：

```cpp
const MissionWaypoint & waypoint = _mission_waypoints[_waypoint_index];
const Eigen::Vector3f target = waypoint.position;
const float horizontal_speed_m_s = horizontalSpeedFor(waypoint);
const float hold_time_s = holdTimeFor(waypoint, is_last_waypoint);
```

2. 用当前位置到目标点的 XY 差值计算航向：

```cpp
const Eigen::Vector2f delta_xy =
  target.head<2>() - _vehicle_local_position->positionNed().head<2>();

if (delta_xy.norm() > 0.5f) {
  heading_rad = std::atan2(delta_xy.y(), delta_xy.x());
}
```

3. 发布目标点：

```cpp
_goto_setpoint->update(
  target,
  heading_rad,
  horizontal_speed_m_s,
  static_cast<float>(_max_vertical_velocity_m_s),
  px4_ros2::degToRad(static_cast<float>(_max_heading_rate_deg_s)));
```

4. 判断是否到达：

```cpp
positionReached(target)
```

到达条件：

```cpp
position_error.norm() < _acceptance_radius_m
&& _vehicle_local_position->velocityNed().norm() < 0.4f
```

也就是说，既要离目标足够近，也要速度足够低，避免高速掠过时误判到达。

## 任务完成逻辑

到达任意航点后都会进入保持逻辑。保持时间来自当前航点的 `hold_time`；如果该航点没有设置 `hold_time`，中间航点默认不等待，最后一个航点默认使用全局 `hold_time_s`。

如果当前航点不是最后一个，保持结束后：

```cpp
++_waypoint_index;
```

如果是最后一个，保持结束后：

```cpp
_state = MissionState::Finished;
completed(px4_ros2::Result::Success);
```
因此，HTML 航点编辑器导出的逐航点参数现在会影响仿真：

```yaml
waypoints:
  - {x: 0.00, y: 0.00, z: -2.00, speed: 0.4, hold_time: 1.0}
  - {x: 2.00, y: 0.00, z: -2.00, speed: 0.8}
  - {x: 2.00, y: 2.00, z: -2.00, hold_time: 3.0}
```

`auto_rtl_after_finish` 当前不会真的发送 RTL 指令，只打印提示。恢复方式仍建议从 QGC 或遥控器切 `Land/RTL/Position`。

## 任务文件解析

任务文件由 `mission_file` 参数指定，例如：

```bash
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py \
  mission_file:=/home/p/px4_ros2_ws/missions/sample_mission.yaml
```

解析函数是 `parseMissionFile()`。它使用 `yaml-cpp` 做结构化 YAML 解析：

1. 用 `YAML::LoadFile()` 读取完整 YAML 文件。
2. 读取顶层任务/避障参数，例如 `max_horizontal_velocity_m_s`、`acceptance_radius_m`、`mission_timeout_s`、`enable_local_avoidance`。
3. 读取 `waypoints` 序列。
4. 从每个航点中提取必填 `x/y/z`，并在存在时提取 `speed/hold_time`。
5. 生成 `MissionWaypoint`。

支持 flow style：

```yaml
waypoints:
  - {x: 0.00, y: 0.00, z: -2.00, speed: 0.4, hold_time: 1.0}
  - {x: 2.00, y: 0.00, z: -2.00, speed: 0.8}
```

也支持 block style：

```yaml
max_horizontal_velocity_m_s: 0.6
acceptance_radius_m: 0.35
mission_timeout_s: 180

waypoints:
  - x: 0.0
    y: 0.0
    z: -2.0
    speed: 0.4
    hold_time: 1.0
  - x: 2.0
    y: 0.0
    z: -2.0
```

生效规则：

- 顶部任务/避障参数会覆盖 launch 或 `start_px4_sim.sh` 传入的同名参数。
- 航点里的 `speed` 会覆盖飞向该航点时的全局水平速度。
- 航点里的 `hold_time` 会覆盖该航点到达后的默认停留时间。
- 文件中的 `z` 是直接使用的 NED 值，不会自动取负。

## mission_waypoints_ned 字符串

除了文件，也可以用参数直接传航点字符串：

```bash
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py \
  mission_waypoints_ned:="0 0 -2, 2 0 -2, 2 2 -2"
```

`parseMissionWaypoints()` 会把这些字符当分隔符：

```text
, ; [ ] ( )
```

然后按每 3 个数一组解析成 x/y/z。

## 运行时目标点

单目标点话题：

```text
/autonomy/target_ned
geometry_msgs/msg/PointStamped
```

示例：

```bash
ros2 topic pub --once /autonomy/target_ned geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'ned'}, point: {x: 2.0, y: 1.0, z: -2.0}}"
```

收到后：

- 保存到 `_manual_target_ned_m`
- 标记 `_manual_target_received = true`
- 清空运行时多航点任务
- 如果模式已激活，立即把当前任务替换成这个单点并重置任务进度

多航点话题：

```text
/autonomy/waypoints_ned
geometry_msgs/msg/PoseArray
```

示例：

```bash
ros2 topic pub --once /autonomy/waypoints_ned geometry_msgs/msg/PoseArray \
"{
  header: {frame_id: 'ned'},
  poses: [
    {position: {x: 0.0, y: 0.0, z: -2.0}},
    {position: {x: 2.0, y: 0.0, z: -2.0}},
    {position: {x: 2.0, y: 2.0, z: -2.0}}
  ]
}"
```

`orientation` 当前不使用。

如果 `accept_runtime_target=false`，这两个运行时输入都会被忽略。

## 避障输入话题

自主模式订阅：

```text
/perception/min_obstacle_distance
/perception/front_obstacle_distance
/perception/left_obstacle_distance
/perception/right_obstacle_distance
/perception/up_obstacle_distance
/perception/down_obstacle_distance
```

类型都是：

```text
std_msgs/msg/Float32
```

单位是米。

当前实际参与水平避障决策的主要是：

- `min_obstacle_distance`
- `front_obstacle_distance`
- `left_obstacle_distance`
- `right_obstacle_distance`

`up/down` 已经接入订阅，但当前主要是给后续三维避障预留。

## 距离数据新鲜度

避障逻辑不会盲目信任旧数据。`freshDistance()` 会检查：

```cpp
if (value <= 0.0f || stamp.nanoseconds() == 0) {
  return std::nullopt;
}

if (_obstacle_data_timeout_s > 0.0 &&
    (node().get_clock()->now() - stamp).seconds() > _obstacle_data_timeout_s) {
  return std::nullopt;
}
```

也就是说：

- 距离必须大于 0
- 必须收到过该话题
- 数据不能超过 `obstacle_data_timeout_s`

过期数据会被忽略。

## 避障状态机

内部状态：

```cpp
enum class MissionState {
  Transit,
  ObstacleHold,
  AvoidingObstacle,
  Aborted,
  Holding,
  Finished
};
```

含义：

- `Transit`
  正常飞向任务航点。
- `AvoidingObstacle`
  正在飞向临时绕行点。
- `ObstacleHold`
  障碍物太近，原地保持。
- `Aborted`
  自主模式失败或超时退出。
- `Holding`
  最后航点到达后的保持。
- `Finished`
  任务完成。

每次 `updateSetpoint()` 中，避障优先于正常任务推进。

## 避障决策顺序

`handleObstacleLogic()` 的大致逻辑是：

```text
如果状态已 Aborted -> 不再控制任务
如果 enable_obstacle_hold=false -> 跳过避障
读取有效最近障碍距离
读取有效前方障碍距离
先尝试 handleLocalAvoidance()
如果前方距离 <= abort 阈值 -> 失败退出
如果前方距离 <= stop 阈值 -> 原地保持
如果之前在保持且障碍清除 -> 恢复 Transit
```

注意局部绕行在 stop/abort 判断之前调用。也就是说，在距离还没近到必须停下前，系统会优先尝试绕行。

建议阈值关系：

```text
avoidance_trigger_distance_m > obstacle_stop_distance_m > obstacle_abort_distance_m
```

例如：

```text
3.0 > 2.0 > 1.0
```

## 局部绕行

触发条件：

```text
enable_local_avoidance = true
front_obstacle_distance <= avoidance_trigger_distance_m
```

如果已经处于 `AvoidingObstacle`，系统会继续飞向 `_avoidance_target_ned_m`。到达临时绕行点后：

```cpp
clearAvoidance();
_state = MissionState::Transit;
```

然后回到原来的任务航点。

如果还没规划绕行点，会调用 `planAvoidanceTarget()`。

规划步骤：

1. 取当前位置。
2. 取当前任务航点。
3. 用“当前位置 -> 当前任务航点”的 XY 方向作为前进方向。
4. 如果离任务点太近，就用当前航向作为前进方向。
5. 读取左/右距离。
6. 判断哪一侧满足 `avoidance_clearance_m`。
7. 如果两侧都可行，选距离更大的一侧。
8. 生成临时目标：

```text
target = current_position
target.xy += forward * avoidance_forward_offset_m
target.xy += lateral * avoidance_lateral_offset_m
target.z = mission_target.z
```

然后用较慢速度飞向临时点：

```cpp
publishGoto(*_avoidance_target_ned_m, _avoidance_speed_m_s);
```

如果左右都不够宽，局部绕行失败，系统会退回到保持/退出逻辑。

## 原地保持

当：

```text
front_distance <= obstacle_stop_distance_m
```

系统进入 `ObstacleHold`：

```cpp
const Eigen::Vector3f hold_target = _vehicle_local_position->positionNed();
_goto_setpoint->update(hold_target, std::nullopt, 0.0f, 0.0f, heading_rate);
```

这里目标点就是当前位置，水平和垂直速度上限为 0。

如果保持时间超过：

```text
obstacle_hold_timeout_s
```

模式会以 `Timeout` 结束：

```cpp
completed(px4_ros2::Result::Timeout);
```

## 强制退出

当：

```text
front_distance <= obstacle_abort_distance_m
```

系统进入 `Aborted` 并调用：

```cpp
completed(px4_ros2::Result::ModeFailureOther);
```

这不会自动降落，也不会自动 RTL。它只是结束 ROS 2 自主模式。恢复动作仍应通过 PX4/QGC/遥控器完成。

## 感知工具包 px4_obstacle_tools

这个包的作用是把不同来源的感知数据统一成简单距离话题。

### obstacle_distance_sim.py

用于无传感器测试。

输出：

```text
/perception/min_obstacle_distance
std_msgs/msg/Float32
```

常见模式：

- `safe`: 发布安全距离。
- `hold`: 发布会触发保持的距离。
- `abort`: 发布会触发退出的距离。
- `wave`: 在安全/危险距离间周期变化。

### pointcloud_min_distance.py

输入：

```text
sensor_msgs/msg/PointCloud2
```

输出：

```text
std_msgs/msg/Float32
```

处理流程：

1. 遍历点云中的 x/y/z。
2. 用 ROI 参数过滤点：

```text
min_x_m <= x <= max_x_m
min_y_m <= y <= max_y_m
min_z_m <= z <= max_z_m
```

3. 按 `distance_mode` 计算距离。
4. 发布最近距离。

`distance_mode` 支持：

- `euclidean`: 三维欧氏距离。
- `x`: 使用 `abs(x)`。
- `y`: 使用 `abs(y)`。
- `z`: 使用 `abs(z)`。

### gz_six_direction_distance.py

用于 Gazebo Classic rplidar scan。

它通过 `gz_topic_reader.sh` 读取 Gazebo transport scan 文本输出，解析 ranges，然后按角度分桶：

- front: 0 度附近
- back: 180 度附近
- left: 90 度附近
- right: -90 度附近

同时结合 `/fmu/out/vehicle_local_position_v1` 推算：

- down: 到地面距离
- up: 到房间顶部距离

输出：

```text
/perception/min_obstacle_distance
/perception/front_obstacle_distance
/perception/back_obstacle_distance
/perception/left_obstacle_distance
/perception/right_obstacle_distance
/perception/up_obstacle_distance
/perception/down_obstacle_distance
/perception/six_direction_distance
```

还可以发布 PX4 原生接口：

```text
/fmu/in/obstacle_distance
px4_msgs/msg/ObstacleDistance
```

当前自主模式主要使用 `/perception/...`，PX4 原生 `ObstacleDistance` 更像预留接口。

## autonomy_stack.launch.py

这个 launch 文件是 ROS 2 自主栈的组合入口。

它总是启动：

```text
px4_autonomy_mode
```

可选启动：

- `livox_ros_driver2_node`
- `obstacle_distance_sim`
- `gz_six_direction_distance`
- `pointcloud_min_distance` 的多个 ROI 实例

参数会被传入 `px4_autonomy_mode`：

```python
Node(
    package="px4_autonomy_mode",
    executable="px4_autonomy_mode",
    name="px4_autonomy_mode",
    parameters=[{
        "mission_size_m": mission_size_m,
        "mission_altitude_m": mission_altitude_m,
        ...
    }],
)
```

这也是为什么新增 C++ 参数时，通常需要同时改两个地方：

1. `autonomy_mode.hpp` 中 `declare_parameter/get_parameter`
2. `autonomy_stack.launch.py` 中 `DeclareLaunchArgument` 和 node parameters

如果还希望一键脚本也能覆盖它，还要改：

3. `scripts/start_px4_sim.sh`

## start_px4_sim.sh

这个脚本负责一键拉起本地仿真环境。它不是简单调用一个 launch，而是生成 `.runtime` 目录下的一组临时脚本。

主要流程：

1. 读取环境变量，例如 `PX4_MODEL`、`PX4_WORLD`、`MISSION_FILE`、避障阈值等。
2. 检查 PX4 目录、工作区 build 结果、QGC 路径。
3. 同步 `sim/worlds/*.sdf` 到 PX4 的 Gazebo world 目录。
4. 生成 `.runtime/ros2_autonomy_env.sh`，统一 source ROS 和工作区。
5. 生成 `.runtime/start_ros2_autonomy.sh`，把环境变量转成 launch 参数。
6. 生成 `.runtime/wait_for_fmu.sh`，等 `/fmu/...` 话题出现后再启动自主模式。
7. 生成 PX4、Gazebo GUI、Agent、QGC 的启动脚本。
8. 用 `gnome-terminal` 启动运行界面。默认是一个窗口内多个标签页，也可通过 `TERMINAL_LAYOUT=windows` 切回多窗口。

日志通常在：

```text
.runtime/logs/
```

这个结构方便调试，因为 PX4、Gazebo、QGC、ROS 2 都有独立标签页和日志。

## 常用调试路径

确认 PX4 和 ROS 2 DDS 通了：

```bash
ros2 topic list | grep '^/fmu/'
```

确认自主节点存在：

```bash
ros2 node list | grep px4_autonomy
```

查看参数：

```bash
ros2 param list /px4_autonomy_mode
ros2 param get /px4_autonomy_mode enable_local_avoidance
```

查看本地位置：

```bash
ros2 topic echo /fmu/out/vehicle_local_position_v1
```

查看障碍物距离：

```bash
ros2 topic echo /perception/front_obstacle_distance
ros2 topic echo /perception/min_obstacle_distance
```

发送目标点：

```bash
ros2 topic pub --once /autonomy/target_ned geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'ned'}, point: {x: 1.0, y: 1.0, z: -2.0}}"
```

查看 Gazebo scan 话题：

```bash
gz topic -l | grep scan
```

查看一键脚本日志：

```bash
ls -lh /home/p/px4_ros2_ws/.runtime/logs
tail -f /home/p/px4_ros2_ws/.runtime/logs/px4.log
```

## 修改代码时的建议路线

### 增加一个自主参数

通常要改：

1. `autonomy_mode.hpp`
   增加成员变量、`declare_parameter`、`get_parameter` 和使用逻辑。
2. `autonomy_stack.launch.py`
   增加 `DeclareLaunchArgument`，并传给 node。
3. `start_px4_sim.sh`
   增加环境变量，并写入 `.runtime/start_ros2_autonomy.sh`。
4. 根 README 或本文档
   更新参数说明。

### 增加一种任务输入

优先考虑改：

- `Px4AutonomyMode` 构造函数中增加订阅。
- 回调里保存任务数据。
- `buildMission()` 中定义优先级。
- `resetMissionProgress()` 中重置对应状态。

### 改避障策略

优先看这些函数：

- `handleObstacleLogic()`
- `handleLocalAvoidance()`
- `planAvoidanceTarget()`
- `publishGoto()`
- `freshDistance()`
- `effectiveObstacleDistance()`

建议保持一个原则：避障逻辑如果接管了目标点，就返回 `true`，让本轮控制循环不要继续执行正常航点推进。

### 接入新传感器

推荐不要一开始就改自主模式核心。先写一个转换节点，把新传感器输出统一成：

```text
std_msgs/msg/Float32
/perception/front_obstacle_distance
/perception/left_obstacle_distance
/perception/right_obstacle_distance
```

等距离话题稳定后，再考虑扩展 `Px4AutonomyMode` 的策略。

## 当前运行时实现的几个边界

- `parseMissionFile()` 已经使用 `yaml-cpp` 完整解析任务 YAML，顶层常用任务/避障参数和逐航点 `speed/hold_time` 都会生效。
- YAML 顶层参数目前覆盖的是代码中已显式支持的字段；如果新增自定义字段，还需要在 `applyMissionFileParameters()` 中接入。
- 每个航点的独立速度、停留时间已经进入 `Px4AutonomyMode`；动作类型还没有进入航点推进逻辑。
- 运行时 `PoseArray` 只能携带位置，不携带逐航点速度/停留时间。
- `auto_rtl_after_finish` 当前不真正切 RTL。
- `up/down` 距离暂未纳入三维绕行。
- 局部绕行是规则式临时航点，不是全局路径规划。
- 没有任务状态发布话题，当前主要靠日志观察。

这些边界不是坏事。当前版本更像一个清晰、可接管、便于扩展的自主模式骨架。

## 推荐阅读顺序

第一次读代码可以按这个顺序：

1. `src/px4_autonomy_mode/src/main.cpp`
2. `src/px4_autonomy_mode/include/px4_autonomy_mode/autonomy_mode.hpp`
3. `src/px4_autonomy_bringup/launch/autonomy_stack.launch.py`
4. `scripts/start_px4_sim.sh`
5. `src/px4_obstacle_tools/px4_obstacle_tools/pointcloud_min_distance.py`
6. `src/px4_obstacle_tools/px4_obstacle_tools/gz_six_direction_distance.py`
7. `missions/sample_mission.yaml`
8. `tools/waypoint_editor.html`
