# ROS 2 项目流程

### 配置环境变量

`source /opt/ros/humble/setup.bash`

### 创建工作空间

`mkdir -p ~/colcon_ws/src  # 创建工作空间及源码目录 `

`cd ~/colcon_ws/src        # 进入源码目录`

### 创建功能包

`ros2 pkg create gazebo_test \  `

​	`--build-type ament_python \  `

​	`--dependencies gazebo_ros xacro robot_state_publisher controller_manager 	mecanum_drive_controller teleop_twist_keyboard \  `

​	`--description "Gazebo simulation for mecanum robot"`

> dependencies 用于声明项目依赖，自动生成 package.xml, setup.py, CMakeLists.txt

### 添加其他项目文件

`gazebo_test/ ├── launch/                  # 存放launch文件 `

​			   `│   └── gazebo_sim_launch.py # 主启动文件（控制整个仿真流程） `

​			   `├── models/                     # 存放仿真资源 `

​			   `│   ├── empty_ground.world   # Gazebo世界文件 `

​			   `│   ├── robot.xacro          # 机器人模型（URDF参数化文件） `

​			   `│   └── robot_control.yaml   # 控制器配置（ros2_control参数） `

​			   `├── gazebo_test/             # Python模块目录（含__init__.py） `

​			   `├── package.xml              # 功能包清单（依赖声明） `

​			   `├── setup.py                 # 安装配置（指定文件安装路径） `

​			   `└── CMakeLists.txt           # 编译配置`

### 核心文件补充配置

setup.py中补充launch文件和模型文件路径：

`data_files=[    `

​	`# ...其他配置...    `

​	`(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),    `

​	`(os.path.join('share', package_name, 'models'), glob('models/*')), `

`],`

### 编译工作空间

`cd ~/colcon_ws `

`colcon build  # 编译所有功能包 `

`source install/setup.bash  # 激活工作空间环境`

### 运行项目

`ros2 launch gazebo_test gazebo_sim_launch.py`