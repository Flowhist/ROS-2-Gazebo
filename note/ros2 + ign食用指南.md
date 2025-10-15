# ROS 2 / Gazebo相关指令

> 基于版本：ubuntu 22.04 + ros 2 humble desktop + gazebo fortress
>



### Gazebo模型格式

**urdf：**ROS 的原生的机器人描述格式，定义连杆link、关节joint、传感器sensor、视觉外观visual、碰撞体collision…

**sdf：**Gazebo 的通用场景描述格式，可描述具体物体的参数，也可描述仿真环境的参数

**xacro：**自带宏定义的xml，urdf的简化表达，使用时一般先通过 `xacro xxx.xacro > xxx.sdf`转成sdf形式

**world：**Gazebo 的专用场景文件，sdf的扩展文件，定义全局参数如引擎类型、时间步长……一般使用<model>标签包含其他单个物体模型，再通过gazebo启动world文件实现所有物体场景的加载

---



### ROS 2 与 Gazebo节点

#### **Gazebo**

列出所有节点：`ign topic -l`

查看节点类型：`ign topic -i -t /clock`

> --topic 表示后接具体节点名

查看节点信息：`ign topic -e -t /clock`

#### ROS 2

列出所有节点：`ros2 topic list`

查看节点状态：`ros2 topic info /clock (-v)`

订阅节点（查看信息）：`ros2 topic echo /tf_static (--once)`

查看节点信息类型定义：`ros2 interface show tf2_msgs/msg/TFMessage`

---



### ROS 与 Gazebo 间通信

https://gazebosim.org/docs/fortress/ros2_integration/

**桥接指令：**`ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG@IGN_MSG`

**用法：**

- `/TOPIC`：gz 内部的话题名称（需与双方订阅 / 发布的话题一致）；
- `ROS_MSG`：ROS 2 的消息类型（如`std_msgs/msg/Int32`、`sensor_msgs/msg/LaserScan`）；
- `IGN_MSG`：gz 的消息类型（如`ignition.msgs.Int32`、`ignition.msgs.LaserScan`）。

> e.g. 
>
> ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist

**作用：**

数据双向 / 单向通信：

- 收集ros2控制指令传给gazebo ignition，驱动机器人执行动作
- 采集gz ignition中传感器数据，同步到ros2，供后续算法处理

---



### Gazebo 机器人控制

https://zhuanlan.zhihu.com/p/669562511

1. 使用下指令打开demo场景

   ```shell
   ign gazebo -v 4 -r visualize_lidar.sdf
   ```

2. 使用下指令桥接ros2指令与gazebo机器人控制指令（ros单向到gz）

   ```shell
   source /opt/ros/humble/setup.bash
   ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
   ```

3. 使用下指令桥接gazebo机器人lidar数据到ros2里（gz单向到ros），同时对lidar数据重映射为lidar_scan和rviz2里的lidarscan可视化部件保持一致

   ```shell
   source /opt/ros/humble/setup.bash
   ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan
   ```

4. 使用以下指令运行ros2的keyboard输入程序并将keyboard输入从 /cmd_vel 重映射 机器人的控制topic

   ```shell
   source /opt/ros/humble/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel
   ```

   后续在终端里即可控制机器人运动

5. 使用下指令启动 Rviz2 可视化lidar数据

   ```shell
   source /opt/ros/humble/setup.bash
   rviz2
   ```

   在rviz2中设置fixed frame为vehicle_blue/lidar_link/gpu_lidar（与gz里的部件名字保持一致），再添加laserscan可视化，即可看到lidar数据

> 上述的控制中使用的keyboard进行，实际算法控制中更多参考：
>
> ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "linear: { x: 0.1 }"

---



### SDF文件配置与定义

https://www.ncnynl.com/archives/202201/4925.html

- <physics></physics>为 **物理** 标签，定义仿真环境的物理属性
- <gui></gui>为 **GUI** 标签，负责gazebo界面设置
- <World control></World control>为 **世界控制** 标签，负责gazebo中世界控制部分的设置
- <World stats></World stats>为 **世界数据** 标签，负责gazebo中世界数据部分的设置
- <light></light>为 **光照** 标签，负责仿真环境的光照设置

......

- <model></model>为 **模型** 标签，定义统一属性如位置
- <link></link>为 **模型内部的实体** 标签，定义具体属性如摩擦、大小
- <joint></joint>为 **链接** 标签，负责连接各个模型
- <plugin></plugin>为 **插件** 标签，sdf文件里的插件类似c文件中的include

......

- <include></include>为 **引用** 标签，用来引用其他地方的sdf文件

所有标签均位于<world></world>里

---

