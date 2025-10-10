# Robo Control - ROS2 SLAM Robot Package

一个基于 ROS2 Humble 和 Ignition Gazebo Fortress 的 SLAM 机器人仿真包，集成了 Google Cartographer 2D SLAM 功能。

## 📋 功能特性

- 🚗 **四轮差动驱动机器人仿真**
- 🗺️ **Google Cartographer 2D SLAM**
- 📡 **多传感器融合**：激光雷达 + IMU + 里程计
- 🎮 **键盘控制**：方向键控制机器人移动
- 🔄 **实时建图和定位**
- 📊 **RViz 可视化**

## 🛠️ 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **仿真器**: Ignition Gazebo Fortress
- **SLAM**: Google Cartographer

## 📦 依赖包

```bash
# ROS2 核心包
sudo apt install ros-humble-desktop

# Ignition Gazebo
sudo apt install ros-humble-ros-gz

# Cartographer SLAM
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

# 导航功能包
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## 🚀 快速开始

### 1. 克隆并编译

```bash
# 创建工作空间
mkdir -p ~/ign_ws/src
cd ~/ign_ws/src

# 克隆项目
git clone <your-repo-url> robo_ctrl

# 编译
cd ~/ign_ws
colcon build --packages-select robo_ctrl
source install/setup.bash
```

### 2. 启动仿真

```bash
# 终端 1: 启动 Ignition Gazebo 仿真
ros2 launch robo_ctrl simulation.launch.py

# 终端 2: 启动 Cartographer SLAM
ros2 launch robo_ctrl cartographer.launch.py

# 终端 3: 启动 RViz 可视化
rviz2 -d ~/ign_ws/src/robo_ctrl/config/cartographer.rviz
```

### 3. 控制机器人

使用键盘方向键控制机器人：
- ⬆️ **向前移动**
- ⬇️ **向后移动**  
- ⬅️ **左转**
- ➡️ **右转**

## 🏗️ 项目结构

```
robo_ctrl/
├── config/                    # 配置文件
│   ├── cartographer_2d.lua   # Cartographer SLAM 配置
│   ├── cartographer.rviz     # RViz 配置
│   └── laser_scan.rviz       # 激光扫描可视化配置
├── launch/                    # 启动文件
│   ├── simulation.launch.py  # 仿真启动
│   ├── cartographer.launch.py # SLAM 启动
│   └── main.launch.py        # 主启动文件
├── models/                    # 3D 模型
│   ├── vehicle_robot/        # 机器人模型
│   │   ├── robot.sdf        # 机器人 SDF 描述
│   │   ├── robot.urdf       # 机器人 URDF 描述
│   │   └── model.config     # 模型配置
│   ├── world.sdf            # 世界场景
│   └── actor.sdf            # 动态障碍物
├── scripts/                   # Python 脚本
│   └── lidar_node.py         # 激光雷达节点
└── README.md                 # 项目说明
```

## 🤖 机器人规格

### 传感器配置
- **激光雷达**: GPU LiDAR, 360°扫描, 360个采样点, 10Hz
- **IMU**: 100Hz 更新频率，带噪声配置
- **里程计**: 差动驱动，50Hz 发布频率
- **相机**: (可选)

### 物理参数
- **尺寸**: 2.0m × 1.0m × 0.5m
- **轮距**: 1.2m
- **轮子半径**: 0.4m
- **质量**: 30kg

### 坐标系层级
```
map (全局地图)
└── vehicle_robot/odom (里程计)
    └── vehicle_robot/chassis (底盘)
        ├── vehicle_robot/imu_link (IMU传感器)
        └── vehicle_robot/lidar_link (激光雷达)
```

## ⚙️ 配置说明

### Cartographer 参数调优
- **建图分辨率**: 0.05m
- **子地图大小**: 45个激光扫描
- **回环检测阈值**: 0.75 (提高以减少误判)
- **IMU 重力时间常数**: 10秒
- **运动过滤器**: 角度 0.5°, 距离 0.05m

### 测试环境
- **六边形竞技场**: 8米半径，6面墙
- **非对称地标**: 红色方块、绿色L形、蓝紫青柱子群
- **静态障碍物**: 6个彩色圆柱体

## 🐛 故障排除

### 常见问题

1. **地图不显示**
   ```bash
   # 检查 Cartographer 节点
   ros2 node list | grep cartographer
   ```

2. **TF 变换错误**
   ```bash
   # 查看 TF 树
   ros2 run tf2_tools view_frames
   ```

3. **传感器数据缺失**
   ```bash
   # 检查话题
   ros2 topic list
   ros2 topic hz /laser_scan
   ros2 topic hz /imu
   ```

### 性能优化

- 减少激光扫描采样点数 (640 → 360)
- 调整地图发布频率 (publish_period_sec = 0.05)
- 降低里程计采样率 (odometry_sampling_ratio = 0.5)

## 📈 开发历程

详细的开发过程和技术细节请参考 [CARTOGRAPHER_GUIDE.md](CARTOGRAPHER_GUIDE.md)

## 🤝 贡献

欢迎提交 Issues 和 Pull Requests！

## 📄 许可证

MIT License - 详见 LICENSE 文件

## 📞 联系方式

如有问题或建议，请通过 GitHub Issues 联系。

---

**享受你的 SLAM 机器人之旅！** 🚀🤖