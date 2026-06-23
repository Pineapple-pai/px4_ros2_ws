# FAST-LIO2 集成技术方案

> 基于 hku-mars/FAST-LIO2 (https://github.com/hku-mars/FAST-LIO2)  
> 目标：在 px4_ros2_ws 项目中集成 FAST-LIO2，实现 3D LiDAR-IMU 定位建图，为 3D 避障/规划提供地图与里程计基础。

---

## 目录

1. [架构总览](#1-架构总览)
2. [FAST-LIO2 简介与选择理由](#2-fast-lio2-简介与选择理由)
3. [包结构设计](#3-包结构设计)
4. [Topic 对接表](#4-topic-对接表)
5. [安装与编译](#5-安装与编译)
6. [launch 文件设计](#6-launch-文件设计)
7. [IMU-LiDAR 标定流程](#7-imu-lidar-标定流程)
8. [室内外切换场景适配](#8-室内外切换场景适配)
9. [集成到 px4_autonomy_mode](#9-集成到-px4_autonomy_mode)
10. [验证方案](#10-验证方案)
11. [常见问题与排障](#11-常见问题与排障)

---

## 1. 架构总览

集成后整体数据流：

```
┌──────────────────────────────────────────────────────────────────┐
│                          FAST-LIO2 核心                          │
│                                                                  │
│  Livox MID360 ──→ LiDAR 点云 (livox/points)                     │
│       +          +                                               │
│  BMI088 IMU ───→ IMU 数据 (/imu)                                │
│       ↓                                                         │
│  FAST-LIO2 前端 (特征提取 + 畸变去除)                              │
│       ↓                                                         │
│  FAST-LIO2 后端 (IESKF 状态估计)     ──→ /Odometry (nav_msgs)   │
│       +                                      +                   │
│  增量式地图更新                          /fast_lio/mapping       │
│  (ikd-Tree)                               (PointCloud2 全局地图) │
└──────────────────────────────────────────────────────────────────┘
       ↓                               ↓
  ┌──────────────────┐      ┌──────────────────────────┐
  │  px4_autonomy_mode│      │ 3D 局部规划器              │
  │  · 使用 FASt-LIO2 │      │ (ego-planner / fast-planner)│
  │    里程计替代PX4  │      │ 读取全局地图 → 规划路径     │
  │    定位           │      │ 输出 GotoSetpoint          │
  └──────────────────┘      └──────────────────────────┘
```

**核心思想**：FAST-LIO2 同时输出：
1. `/Odometry` — 高频（~100Hz）里程计，替代 PX4 视觉/INS 定位
2. `/fast_lio/mapping` — 低频（~1Hz）增量更新全局点云地图，供规划器使用

---

## 2. FAST-LIO2 简介与选择理由

### 2.1 技术原理

FAST-LIO2 (Fast LiDAR-Inertial Odometry) 是港大 hku-mars 团队的开源项目，核心特点：

| 特性 | 说明 |
|------|------|
| 传感器 | LiDAR + IMU，无需其他传感器 |
| 核心算法 | 迭代误差状态卡尔曼滤波 (IESKF) |
| 地图结构 | ikd-Tree (增量式 kd-tree)，支持高效动态增删点 |
| 无需初始 | 无需初始位姿，自动初始化 |
| 计算效率 | 单核 CPU 实时，无需 GPU |
| 输出频率 | 里程计 ~100Hz / 地图 ~1Hz |

### 2.2 为什么选 FAST-LIO2 而非 Point-LIO

| 对比项 | FAST-LIO2 | Point-LIO |
|--------|-----------|-----------|
| 社区成熟度 | ⭐⭐⭐⭐⭐ 成熟，ROS2版本稳定 | ⭐⭐⭐ 较新，ROS2版可能不完善 |
| 文档/教程 | 丰富 (GitHub 10k+ star) | 较少 |
| 室内复杂场景 | ⭐⭐⭐ 好 | ⭐⭐⭐ 好 |
| 室外高速 | ⭐⭐⭐ 好 | ⭐⭐⭐⭐ 更好 |
| 调试难度 | 低 | 中 |
| IMU初始化 | 支持自动初始化 | 需要较好初始值 |

**选择结论**：室内外切换 + 首次集成 → **FAST-LIO2** 是更稳妥的选择。后期可迁移到 Point-LIO 或 R3LIVE++。

---

## 3. 包结构设计

本项目内新增一个 ROS2 包 `px3_fastlio_bridge`，用于对接 FAST-LIO2 与本项目的规划/控制层。

```
src/
├── fast_lio                          # FAST-LIO2 原始仓库 (submodule 或手动 clone)
│   ├── src/                          # FAST-LIO2 核心源码
│   ├── launch/                       # FAST-LIO2 自带 launch（可能需修改）
│   └── config/                       # Livox MID360 + IMU 配置文件
│
└── px4_fastlio_bridge/               # 【新增】对接桥接层
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   ├── fastlio_mapping.launch.py # 启动 FAST-LIO2 + 桥接
    │   └── fastlio_planning.launch.py# 启动 FAST-LIO2 + 规划器
    ├── config/
    │   ├── mid360_imu.yaml           # Livox MID360 内置 IMU 参数
    │   └── fastlio_params.yaml       # FAST-LIO2 参数覆盖
    └── src/
        ├── lio_odometry_bridge.cpp   # FAST-LIO2 里程计 → px4_autonomy_mode 接口
        ├── pointcloud_to_costmap.cpp # 点云 → 2D/3D 代价地图 (可选)
        └── dynamic_obstacle_tracker.cpp # 动态障碍物追踪
```

### 包依赖关系

```
px4_fastlio_bridge
  ├── fast_lio (编译依赖: fast_lio 库)
  ├── livox_ros_driver2 (运行时驱动)
  ├── px4_autonomy_mode (控制接口)
  └── px4_obstacle_tools (现有距离工具，可选复用)
```

---

## 4. Topic 对接表

### 4.1 FAST-LIO2 所需输入 Topic

| Topic | 类型 | 来源 | 说明 |
|-------|------|------|------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox MID360 驱动 | 原始 LiDAR 点云包 |
| `/imu` | `sensor_msgs/msg/Imu` | PX4 或机载IMU | IMU 加速度/角速度 |

### 4.2 FAST-LIO2 输出 Topic

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/Odometry` | `nav_msgs/msg/Odometry` | ~100Hz | IESKF 估计的位姿+速度 |
| `/fast_lio/mapping` | `sensor_msgs/msg/PointCloud2` | ~1Hz | 全局点云地图 |
| `/fast_lio/aftmapped` | `sensor_msgs/msg/PointCloud2` | ~10Hz | 当前帧匹配后点云 |
| `/fast_lio/cloud_world` | `sensor_msgs/msg/PointCloud2` | ~10Hz | 世界坐标系下的点云 |
| `/fast_lio/state_imu` | `sensor_msgs/msg/Imu` | ~100Hz | IMU 状态估计 |
| `/fast_lio/path` | `nav_msgs/msg/Path` | ~10Hz | 估计轨迹路径 |

### 4.3 桥接层输出

| Topic | 类型 | 说明 |
|-------|------|------|
| `/autonomy/lio_odometry` | `nav_msgs/msg/Odometry` | LIO 里程计 (relay from /Odometry) |
| `/autonomy/lio_position_ned` | `geometry_msgs/msg/PointStamped` | NED 坐标系下的 LIO 位置 |
| `/autonomy/local_map` | `sensor_msgs/msg/PointCloud2` | 局部窗口地图 (供规划器) |
| `/perception/obstacle_clusters` | `px4_fastlio_bridge/msg/ObstacleClusterArray` | 动态障碍物簇 |

### 4.4 坐标系约定

| 坐标系 | 说明 |
|--------|------|
| `livox_frame` | Livox MID360 本体坐标系 (前向 X, 左向 Y, 上向 Z) |
| `map` / `world` | FAST-LIO2 世界坐标系 (第一帧初始化为原点) |
| `body` / `base_link` | 无人机机体坐标系 (前向 X, 右向 Y, 下向 Z) |
| `ned` | PX4 导航坐标系 (北 X, 东 Y, 下 Z) |

**桥接关键**：`map → ned` 坐标变换。FAST-LIO2 默认输出在 `map` 系 (前-左-上)，需转换为 NED (北-东-下) 供 px4_autonomy_mode 使用。

---

## 5. 安装与编译

### 5.1 克隆 FAST-LIO2

```bash
# 在 src/ 目录下
cd ~/px4_ros2_ws/src

# 推荐：作为 git submodule 管理
git submodule add https://github.com/hku-mars/FAST-LIO2.git fast_lio

# 或直接 clone
# git clone https://github.com/hku-mars/FAST-LIO2.git fast_lio

cd fast_lio
git checkout master  # 或指定稳定的 commit/tag
```

### 5.2 安装依赖

```bash
# Eigen3 (通常已安装)
sudo apt install libeigen3-dev

# PCL (点云库)
sudo apt install libpcl-dev

# livox_ros_driver2 (已在项目中)
# 确认 livox_ros_driver2 在 src/ 中

# ROS2 依赖 (Humble)
sudo apt install ros-humble-tf2-eigen ros-humble-pcl-conversions
```

### 5.3 编译

```bash
cd ~/px4_ros2_ws
colcon build --packages-select fast_lio livox_ros_driver2
source install/setup.bash
```

**注意**：
- FAST-LIO2 的 CMakeLists.txt 可能需要小幅修改以适配 ROS2 Humble
- 确认 `package.xml` 中的 `<build_depend>` 和 `<exec_depend>` 正确

### 5.4 修改 FAST-LIO2 配置文件

复制 FAST-LIO2 的示例配置并进行修改：

```bash
mkdir -p ~/px4_ros2_ws/src/px4_fastlio_bridge/config
cp ~/px4_ros2_ws/src/fast_lio/config/avia.yaml \
   ~/px4_ros2_ws/src/px4_fastlio_bridge/config/mid360.yaml
```

修改 `mid360.yaml` 中的关键参数：

```yaml
# 点云参数
point_filter_to: 4  # 降采样（越大点越稀疏，计算越快）
feature_extract_enable: false  # 初期关掉特征提取，用全部点

# LiDAR 参数
lid_type: "livox_mid360"  # 或 livox_avia
scan_rate: 10              # MID360 默认 10 Hz
time_unit: 0.125           # 纳秒单位 (MID360)

# 滤波参数
filter_size_map_min: 0.5   # 地图体素滤波大小 (米)
filter_size_surf: 0.5

# IMU 参数
imu_enable: true
# MID360 内置 IMU 参数见下一节
```

---

## 6. launch 文件设计

### 6.1 基础 launch：仅启动 FAST-LIO2

`src/px4_fastlio_bridge/launch/fastlio_mapping.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_fast_lio = get_package_share_directory("fast_lio")
    pkg_bridge = get_package_share_directory("px4_fastlio_bridge")
    pkg_livox = get_package_share_directory("livox_ros_driver2")

    default_fastlio_config = os.path.join(
        pkg_bridge, "config", "mid360.yaml")
    default_livox_config = os.path.join(
        pkg_livox, "config", "MID360_config.json")

    return LaunchDescription([
        DeclareLaunchArgument("livox_config", default_value=default_livox_config),
        DeclareLaunchArgument("fastlio_config", default_value=default_fastlio_config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # Livox MID360 驱动
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "user_config_path": LaunchConfiguration("livox_config"),
            }],
        ),

        # FAST-LIO2 (使用自带的 launch)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_fast_lio, "launch", "mapping_360.launch.py")
            ]),
            launch_arguments={
                "config_file": LaunchConfiguration("fastlio_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),

        # 桥接节点：LIO 里程计 → px4_autonomy_mode
        Node(
            package="px4_fastlio_bridge",
            executable="lio_odometry_bridge",
            name="lio_odometry_bridge",
            output="screen",
            parameters=[{
                "input_odom_topic": "/Odometry",
                "output_position_topic": "/autonomy/lio_position_ned",
                "output_odom_topic": "/autonomy/lio_odometry",
            }],
        ),
    ])
```

### 6.2 完整 launch：FAST-LIO2 + 3D 规划器

`src/px4_fastlio_bridge/launch/fastlio_planning.launch.py`:

```python
# 包含 fastlio_mapping.launch.py +
# ego-planner / fast-planner / MPPI 局部规划器节点
# + px4_autonomy_mode 节点(使用 LIO 里程计)
# 
# 详细内容取决于选定的规划器
```

### 6.3 在现有启动栈中集成

修改 `src/px4_autonomy_bringup/launch/autonomy_stack.launch.py` 增加参数：

```python
# 新增参数
DeclareLaunchArgument("use_fastlio", default_value="false",
                      description="Use FAST-LIO2 for localization and mapping"),
DeclareLaunchArgument("fastlio_config", default_value=...),
```

并在 `Node` 列表中条件式包含 fastlio 节点。

---

## 7. IMU-LiDAR 标定流程

### 7.1 为什么需要标定

FAST-LIO2 依赖准确的 LiDAR-IMU 外参（旋转矩阵 + 平移向量）和 IMU 内参（噪声密度、零偏）。对于 Livox MID360（内置 BMI088 IMU），标定步骤：

### 7.2 外参标定（LiDAR 到 IMU）

使用 `li_calib` 或 `lidar_IMU_calib` 工具：

```bash
# 1. 克隆标定工具
cd ~/px4_ros2_ws/src
git clone https://github.com/APRIL-ZJU/lidar_IMU_calib.git

# 2. 准备标定数据
#    手持设备在特征丰富的环境中缓慢运动，录制 rosbag
ros2 bag record -o calib_data /livox/lidar /imu

# 3. 运行标定
#    按工具文档执行
```

**录制标定数据的建议**：
- 在室内有墙角、柱子、桌椅等特征的环境
- 缓慢运动，包括 6 自由度（平动 + 旋转）
- 持续 2~3 分钟
- 避免快速旋转导致点云严重畸变

### 7.3 IMU 内参标定

```bash
# 使用 imu_utils 或 allan_variance_ros
# 录制静止 1~2 小时的 IMU 数据
ros2 bag record -o imu_static /imu

# 运行 Allan 方差分析得到 IMU 噪声参数
```

**快速预估值**（Livox MID360 内置 BMI088）：

| 参数 | 典型值 |
|------|--------|
| 加速度计噪声密度 | 0.002 m/s²/√Hz |
| 加速度计零偏不稳定性 | 0.0003 m/s² |
| 陀螺噪声密度 | 0.0015 rad/s/√Hz |
| 陀螺零偏不稳定性 | 0.0001 rad/s |
| LiDAR-IMU 旋转 (外参) | 近似单位阵 (MID360 本体集成) |
| LiDAR-IMU 平移 (外参) | 近似 0 (已知偏移量写入yaml) |

### 7.4 标定结果写入配置

```yaml
# mid360.yaml 中的 IMU 参数部分
# Extrinsic parameter between IMU and LiDAR
extrinsic_rotation: [1, 0, 0,
                     0, 1, 0,
                     0, 0, 1]   # 近似单位阵
extrinsic_translation: [0.0, 0.0, 0.0]  # 近似零

# IMU noise parameters
imu_gyro_noise: 0.0015
imu_acc_noise: 0.002
imu_gyro_bias_n: 0.0001
imu_acc_bias_n: 0.0003
imu_gyro_w: 0.0001
imu_acc_w: 0.0003
```

---

## 8. 室内外切换场景适配

### 8.1 场景特征分析

| 特征 | 室内 | 室外 |
|------|------|------|
| 点云特征 | 丰富（墙、柱、角点） | 稀疏（树木、建筑） |
| 光照 | 无关（LiDAR 不依赖光） | 无关 |
| GPS | 无 | 有 |
| 典型距离 | 3~20m | 10~100m+ |
| 动态物体 | 人、门 | 车、行人 |

### 8.2 参数自适应策略

建议使用两套参数配置，通过场景感知自动切换：

```yaml
# config/mid360_indoor.yaml (室内)
point_filter_to: 2         # 用更多点，特征更丰富
filter_size_map_min: 0.3   # 更精细的地图
filter_size_surf: 0.3

# config/mid360_outdoor.yaml (室外)
point_filter_to: 6         # 降采样更多，提计算效率
filter_size_map_min: 0.8   # 更粗的地图
filter_size_surf: 0.8
max_range: 80.0            # 室外更远
```

**自动切换触发条件**：
1. 检测到 GPS 有效 → 室外模式
2. 最近帧平均点云距离 < 15m → 室内模式
3. 统计特征点数量变化

### 8.3 FAST-LIO2 定位退化处理

| 退化场景 | 检测方法 | 应对策略 |
|---------|---------|---------|
| 长廊/隧道 | 协方差矩阵特征值分析 | 减小地图更新速率，保持匀速 |
| 空旷室外 | 最近邻匹配点数不足 | 降低地图更新阈值，容忍更松 |
| 快速旋转 | IMU 预积分方差激增 | 短暂切换预测模式，等待重收敛 |

**实现退化检测**：

```cpp
// 在 lio_odometry_bridge.cpp 中
// 通过监听 /fast_lio/state_imu 的协方差估计SLAM退化程度
// 若检测到退化，发布 warning 到 px4_autonomy_mode
// px4_autonomy_mode 据此降低速度或切换安全模式
```

---

## 9. 集成到 px4_autonomy_mode

### 9.1 LIO 里程计桥接节点

`src/px4_fastlio_bridge/src/lio_odometry_bridge.cpp`:

```cpp
/**
 * 功能：
 * 1. 订阅 FAST-LIO2 的 /Odometry (map 系)
 * 2. 转换为 NED 坐标系下的位置/速度
 * 3. 发布为 px4_autonomy_mode 可接收的话题
 * 4. 当 px4_autonomy_mode 启用 LIO 模式时，用此位置替代 PX4 定位
 */

class LioOdometryBridge : public rclcpp::Node
{
public:
  LioOdometryBridge() : Node("lio_odometry_bridge")
  {
    // 订阅 FAST-LIO2 里程计
    _odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        // map → NED 坐标变换
        // 注意：map系是 前-左-上，NED是 北-东-下
        // 对于初始对齐，需要标定 map 与 NED 的旋转关系
        
        // 发布 NED 位置
        auto pos_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
        pos_msg->header = msg->header;
        pos_msg->header.frame_id = "ned";
        pos_msg->point.x = msg->pose.pose.position.x;  // 需变换
        pos_msg->point.y = msg->pose.pose.position.y;
        pos_msg->point.z = -msg->pose.pose.position.z;  // Z轴取反
        _position_pub->publish(std::move(pos_msg));

        // 转发完整里程计
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>(*msg);
        odom_msg->header.frame_id = "ned";
        odom_msg->child_frame_id = "base_link_ned";
        _odom_pub->publish(std::move(odom_msg));
      });

    _position_pub = create_publisher<geometry_msgs::msg::PointStamped>(
      "/autonomy/lio_position_ned", rclcpp::QoS(10));
    _odom_pub = create_publisher<nav_msgs::msg::Odometry>(
      "/autonomy/lio_odometry", rclcpp::QoS(10));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _position_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
};
```

### 9.2 px4_autonomy_mode 修改方案

在 `autonomy_mode.hpp` 中新增（或通过参数切换）：

```cpp
// 新增参数
node.declare_parameter("use_lio_odometry", false);
node.declare_parameter("lio_odometry_topic", "/autonomy/lio_odometry");

// 新增订阅
_lio_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(
    _lio_odometry_topic, rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (_use_lio_odometry) {
        // 更新位置估计（替代 _vehicle_local_position）
        _lio_position_ned = Eigen::Vector3f(
          msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          msg->pose.pose.position.z);
        _lio_velocity_ned = Eigen::Vector3f(
          msg->twist.twist.linear.x,
          msg->twist.twist.linear.y,
          msg->twist.twist.linear.z);
        _lio_position_stamp = node().get_clock()->now();
      }
    });
```

**重要**：这种方式下，无人机定位仍依赖 PX4 EKF2 融合结果。若想让 LIO 里程计直接作为 PX4 EKF 的观测输入（即外部定位源），需要通过 `px4_msgs/msg/VehicleExternalPosition` 或 `px4_msgs/msg/ObstacleDistance` 接口注入。这需要在 `px4_ros2_interface_lib` 层面新增 publisher。

### 9.3 规划器输出对接

不论选择哪种 3D 规划器，其最终输出需要适配 `MulticopterGotoSetpointType`：

```cpp
// 3D 规划器输出的路径点 / 轨迹点
// 以 /autonomy/waypoints_ned (PoseArray) 发送给 px4_autonomy_mode
//
// 或：规划器输出单目标点 /autonomy/target_ned
// 规划器内部持续跟踪下一目标，发布下一点到话题
```

---

## 10. 验证方案

### 10.1 Phase 1：FAST-LIO2 裸跑验证

```bash
# 终端 1: 启动 FAST-LIO2 + Livox 驱动
ros2 launch px4_fastlio_bridge fastlio_mapping.launch.py

# 终端 2: 查看里程计
ros2 topic echo /Odometry --once

# 终端 3: Rviz 可视化
rviz2 -d src/fast_lio/rviz/fast_lio.rviz
```

**验证通过标准**：
- [ ] `/Odometry` 输出稳定，频率 ~100Hz
- [ ] `/fast_lio/mapping` 有增量地图更新
- [ ] 手动缓慢移动 LiDAR，地图清晰无拖影

### 10.2 Phase 2：仿真环境验证

```bash
# 终端 1: Gazebo 仿真 (使用 x500_lidar 模型)
make px4_sitl gazebo-classic_x500_lidar

# 终端 2: MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# 终端 3: 启动完整性栈
ros2 launch px4_autonomy_bringup autonomy_stack.launch.py \
  use_sim_time:=true \
  mission_file:=$(pwd)/missions/sample_mission.yaml
```

**仿真中需修改**：
- 使用 Gazebo 中的 LiDAR scan 点云替代 Livox 数据
- 或安装 Gazebo 的 Livox MID360 插件模拟

### 10.3 Phase 3：真机验证

1. 先在地面手持 LiDAR 录制 rosbag → 离线跑 FAST-LIO2 验证
2. 无人机悬停 → 启动 FAST-LIO2 → 观察里程计稳定性
3. 低速手动飞行 → 对比 PX4 定位与 FAST-LIO2 定位一致性
4. 切换到自主模式，在已知无障区域飞行

---

## 11. 常见问题与排障

### 11.1 编译错误

| 错误 | 原因 | 解决 |
|------|------|------|
| `Eigen3` 未找到 | 未安装 | `sudo apt install libeigen3-dev` |
| `PCL` 未找到 | 未安装 | `sudo apt install libpcl-dev` |
| `livox_ros_driver2` 找不到 | 未编译 | `colcon build --packages-select livox_ros_driver2` |
| `tf2_eigen` 未找到 | 未安装 | `sudo apt install ros-humble-tf2-eigen` |

### 11.2 运行时问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 里程计跳跃/发散 | IMU 标定不准 | 重新标定 IMU 参数 |
| 地图重叠/模糊 | LiDAR-IMU 外参不准确 | 重新标定外参 |
| 初始化失败 (一直 waiting for LiDAR) | 点云话题不匹配 | 检查 `/livox/lidar` 是否存在、`lid_type` 参数 |
| 里程计频率低 | 点云降采样不够 | 增大 `point_filter_to` |
| CPU 占用过高 | 地图体素太小 | 增大 `filter_size_map_min` |
| 室内外切换时定位发散 | 参数不适合当前场景 | 手动切换参数集或实现自动检测 |

### 11.3 ROS2 Humble 兼容性

FAST-LIO2 的原始仓库主要支持 ROS1 (Melodic/Noetic)。ROS2 版本注意事项：

```bash
# 确认使用 FAST-LIO2 的 ROS2 分支
git clone https://github.com/hku-mars/FAST-LIO2.git
cd FAST-LIO2
git checkout ros2  # 或 master + 手动适配

# 如果原仓没有 ros2 分支，可使用社区 fork：
git clone https://github.com/yanjingang/FAST-LIO2.git -b ros2
```

**常见修改点**：
1. `CMakeLists.txt` 中 `find_package` 版本号
2. `tf2` 消息适配
3. 时间戳使用 `rclcpp::Time` 替代 `ros::Time`
4. 参数接口使用 `rclcpp` 替代 `ros::param`

---

## 附录 A：推荐的 3D 规划器对比

| 规划器 | 作者 | 说明 | 适合场景 |
|--------|------|------|---------|
| **ego-planner** | hku-mars | 基于梯度的局部轨迹优化，支持实时避障 | 动态障碍物，室内外 |
| **fast-planner** | hku-mars | B-spline 轨迹 + 动力学可行规划 | 静态环境，复杂3D |
| **motion_primitive** | PX4 | 速度控制原始规划 | 简单避障 |
| **MPPI** | 学术论文 | 模型预测路径积分 | 带碰撞概率的避障 |

**推荐**：先试 `fast-planner`（与 FAST-LIO2 同团队，兼容性好），后升级 `ego-planner`（支持动态避障）。

## 附录 B：参考资源

| 资源 | 链接 |
|------|------|
| FAST-LIO2 GitHub | https://github.com/hku-mars/FAST-LIO2 |
| FAST-LIO2 论文 | Wei Xu, et al. "FAST-LIO2: Fast Direct LiDAR-inertial Odometry" |
| fast-planner | https://github.com/HKUST-Aerial-Robotics/Fast-Planner |
| ego-planner | https://github.com/ZJU-FAST-Lab/ego-planner |
| Livox MID360 规格 | https://www.livoxtech.com/mid360 |
| imu_utils 标定工具 | https://github.com/gaowenliang/imu_utils |
| lidar_IMU_calib | https://github.com/APRIL-ZJU/lidar_IMU_calib |