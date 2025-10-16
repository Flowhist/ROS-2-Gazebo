# Robo Control - ROS2 SLAM 机器人仿真包

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ignition](https://img.shields.io/badge/Ignition-Fortress-orange)](https://gazebosim.org/)
[![Cartographer](https://img.shields.io/badge/SLAM-Cartographer-green)](https://github.com/cartographer-project/cartographer)

一个集成 Google Cartographer 2D SLAM 的 ROS2 机器人仿真项目，支持多传感器融合建图、键盘控制和实时可视化。

## ✨ 核心功能

- 🚗 **四轮差动驱动机器人** - 完整的物理仿真模型
- 🗺️ **实时 2D SLAM 建图** - Google Cartographer 算法
- 📡 **多传感器融合** - 激光雷达 (360°) + IMU + 里程计
- 🎮 **键盘遥控** - 方向键直接控制机器人移动
- 🏟️ **复杂测试环境** - 六边形房间 + 漏斗通道 + 狭长走廊
- 📊 **可视化界面** - RViz2 实时显示建图过程

## 🛠️ 环境要求

| 组件 | 版本 |
|------|------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Ignition Gazebo | Fortress |
| Python | 3.10+ |

## 📦 快速安装

### 1. 安装依赖

```bash
# ROS2 核心
sudo apt update
sudo apt install ros-humble-desktop

# Ignition Gazebo
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge

# Cartographer SLAM
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

# 其他工具
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

### 2. 构建项目

```bash
# 创建工作空间
mkdir -p ~/ign_ws/src
cd ~/ign_ws/src
git clone <your-repo-url> robo_ctrl

# 编译
cd ~/ign_ws
colcon build --packages-select robo_ctrl
source install/setup.bash
```

## 🚀 启动指南

### 方式一：一键启动（推荐）

```bash
# 启动仿真 + SLAM + RViz（全部功能）
ros2 launch robo_ctrl main.launch.py
```

### 方式二：分步启动

```bash
# 终端 1: 启动 Gazebo 仿真
ros2 launch robo_ctrl simulation.launch.py

# 终端 2: 启动 Cartographer SLAM
ros2 launch robo_ctrl cartographer.launch.py

# 终端 3（可选）: 启动 RViz 可视化
rviz2 -d ~/ign_ws/src/robo_ctrl/config/cartographer.rviz
```

### 🎮 控制机器人

使用键盘方向键控制：
- **↑** 前进
- **↓** 后退
- **←** 左转
- **→** 右转
- **空格** 停止

> 💡 提示：在 Gazebo 窗口激活状态下按键盘方向键

## 📁 项目结构

```
robo_ctrl/
├── config/                       # 配置文件
│   ├── cartographer_2d.lua      # Cartographer 参数配置
│   ├── cartographer.rviz        # RViz 可视化配置
│   └── laser_scan.rviz          # 激光扫描显示配置
├── launch/                       # 启动文件
│   ├── main.launch.py           # 🔥 主启动文件（一键启动）
│   ├── simulation.launch.py     # Gazebo 仿真启动
│   └── cartographer.launch.py   # SLAM 算法启动
├── models/                       # 3D 模型资源
│   ├── vehicle_robot/           # 机器人模型
│   │   ├── robot.sdf           # Gazebo 模型描述
│   │   ├── robot.urdf          # ROS 机器人描述
│   │   └── model.config        # 模型元数据
│   ├── world.sdf               # 世界场景（六边形 + 通道）
│   └── actor.sdf               # 动态障碍物（可选）
├── robo_ctrl/                    # Python 功能包
│   └── __init__.py
├── scripts/                      # 脚本工具
├── test/                         # 单元测试
├── package.xml                   # ROS2 包清单
├── setup.py                      # Python 包配置
└── README.md                     # 📖 本文档
```

## 🤖 机器人规格

### 传感器配置

| 传感器 | 规格 | 话题 | 频率 |
|--------|------|------|------|
| 激光雷达 | 360° GPU LiDAR, 360 点 | `/laser_scan` | 30 Hz |
| IMU | 6 轴加速度 + 陀螺仪 | `/imu` | 100 Hz |
| 里程计 | 差动驱动编码器 | `/odom` | 50 Hz |
| 关节状态 | 4 个车轮关节 | `/joint_states` | 30 Hz |

### 物理参数

- **尺寸**: 2.0m (长) × 1.0m (宽) × 0.5m (高)
- **轮距**: 1.2m
- **轮半径**: 0.4m
- **质量**: 30 kg
- **最大速度**: 线速度 2.0 m/s，角速度 1.5 rad/s

### TF 坐标树

```
map (全局地图坐标系)
  └── vehicle_robot/odom (里程计坐标系)
      └── vehicle_robot/chassis (机器人底盘)
          ├── vehicle_robot/imu_link (IMU 中心)
          └── vehicle_robot/lidar_link (雷达中心)
              ├── vehicle_robot/front_left_wheel
              ├── vehicle_robot/front_right_wheel
              ├── vehicle_robot/rear_left_wheel
              └── vehicle_robot/rear_right_wheel
```

## 🏟️ 测试环境

### 六边形房间
- **半径**: 8 米
- **墙高**: 2 米
- **墙厚**: 0.3 米
- **开口**: 正前方（0°）打开连接通道

### 漏斗过渡区
- **功能**: 将六边形 8 米宽开口收窄至 3 米
- **墙体**: 墙2（右前 60°）和墙6（左前 300°）延长
- **长度**: 约 10 米

### 狭长通道
- **尺寸**: 80 米长 × 3 米宽
- **用途**: 测试长距离建图和回环检测

### 障碍物
- 3 个彩色圆柱体（黄、紫、青）
- 红色方形地标
- 绿色 L 型地标
- 蓝紫青三柱群地标

> 💡 设计目的：测试 SLAM 算法在对称环境、狭窄空间和大尺度场景下的性能

## ⚙️ Cartographer 配置

### 关键参数

```lua
-- 传感器采样率
rangefinder_sampling_ratio = 1.0      -- 激光雷达全采样
odometry_sampling_ratio = 0.7         -- 里程计 70% 采样
imu_sampling_ratio = 1.0              -- IMU 全采样

-- 扫描匹配（减少重影）
translation_weight = 10               -- 平移匹配权重
rotation_weight = 35                  -- 旋转匹配权重

-- 子地图配置
num_range_data = 45                   -- 每个子地图 45 帧数据
resolution = 0.05                     -- 5cm 网格分辨率

-- 回环检测（避免误判）
min_score = 0.80                      -- 最小置信度阈值
max_constraint_distance = 15.0        -- 最大搜索距离 15m
sampling_ratio = 0.2                  -- 采样率 20%
```

### 针对重影问题的优化

当在封闭房间扫描时出现边缘重影，已进行如下调整：
- ✅ 提高运动过滤器阈值（过滤小幅移动噪声）
- ✅ 降低扫描匹配权重（减少过度拟合）
- ✅ 启用自适应体素过滤器（减少点云噪声）
- ✅ 提高回环检测阈值（避免误判）
- ✅ 减少回环搜索距离（聚焦局部优化）

## 🐛 故障排除

### 常见问题

#### 1. 地图不显示
```bash
# 检查 Cartographer 节点是否运行
ros2 node list | grep cartographer

# 检查激光雷达数据
ros2 topic echo /laser_scan --once

# 查看占据栅格地图
ros2 topic hz /map
```

#### 2. TF 变换错误
```bash
# 生成 TF 树
ros2 run tf2_tools view_frames

# 查看特定变换
ros2 run tf2_ros tf2_echo map vehicle_robot/odom
```

#### 3. 机器人不移动
```bash
# 检查速度命令话题
ros2 topic echo /cmd_vel

# 确认 Gazebo 桥接正常
ros2 topic list | grep cmd_vel

# 手动发送测试命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

#### 4. RViz 黑屏或无显示
```bash
# 确认 use_sim_time 参数一致
ros2 param get /rviz2 use_sim_time

# 重新加载 RViz 配置
rviz2 -d ~/ign_ws/src/robo_ctrl/config/cartographer.rviz
```

### 性能优化建议

| 问题 | 解决方案 | 配置参数 |
|------|---------|---------|
| CPU 占用高 | 降低激光雷达采样率 | `num_subdivisions_per_laser_scan = 2` |
| 建图延迟大 | 减少子地图发布频率 | `submap_publish_period_sec = 0.1` |
| 内存占用高 | 增加网格分辨率 | `resolution = 0.1` |
| 回环检测慢 | 降低搜索范围 | `max_constraint_distance = 10.0` |

## 📊 ROS2 话题列表

### 发布话题

| 话题 | 消息类型 | 描述 |
|------|---------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 占据栅格地图 |
| `/submap_list` | `cartographer_ros_msgs/SubmapList` | 子地图列表 |
| `/laser_scan` | `sensor_msgs/LaserScan` | 激光扫描数据 |
| `/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/odom` | `nav_msgs/Odometry` | 里程计数据 |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/tf` | `tf2_msgs/TFMessage` | TF 变换 |
| `/clock` | `rosgraph_msgs/Clock` | 仿真时钟 |

### 订阅话题

| 话题 | 消息类型 | 描述 |
|------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制命令 |

## 🎯 使用场景

- ✅ SLAM 算法研究与教学
- ✅ 自主导航算法开发
- ✅ 多传感器融合测试
- ✅ 机器人控制策略验证
- ✅ 路径规划算法评估

## 📚 相关资源

- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [Ignition Gazebo 教程](https://gazebosim.org/docs/fortress/tutorials)
- [Cartographer 官方文档](https://google-cartographer.readthedocs.io/)
- [TF2 坐标变换指南](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

## 🤝 贡献指南

欢迎提交 Issue 和 Pull Request！请遵循以下规范：
1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 👨‍💻 作者

- **Flowhist** - [GitHub](https://github.com/Flowhist)

---

<div align="center">

**⭐ 如果这个项目对你有帮助，请给个 Star！**

🚀 享受你的 SLAM 机器人之旅！🤖

</div>

