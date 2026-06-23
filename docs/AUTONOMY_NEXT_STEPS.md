# ROS2 Autonomy 多航点飞行与避障验证指南

本文档用于指导后续在 `ROS2 Autonomy` 模式下进行多航点航线飞行、局部绕行避障验证，以及如何把多航点航线写成文件长期复用。

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