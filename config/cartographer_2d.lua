include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 机器人的基础坐标系名称（frame id 与 ign gazebo 侧对齐）
  map_frame = "map",
  tracking_frame = "vehicle_robot/imu_link",  -- 跟踪 IMU 坐标系（在旋转中心）
  published_frame = "vehicle_robot/odom",     -- Cartographer 发布 map→odom 的变换
  odom_frame = "vehicle_robot/odom",          -- 里程计坐标系
  
  -- 是否提供里程计数据（设为 false，因为 Ignition 已经提供了 odom→chassis）
  provide_odom_frame = false,
  
  -- 是否发布2d位姿
  publish_frame_projected_to_2d = true,
  
  -- 使用位姿外推器（用于在扫描之间预测位姿）
  use_pose_extrapolator = true,
  
  -- 是否使用里程计数据
  use_odometry = true,
  
  -- 是否使用GPS数据
  use_nav_sat = false,
  
  -- 是否使用路标
  use_landmarks = false,
  
  -- 传感器数量配置
  num_laser_scans = 1,         -- 使用1个激光雷达
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,        -- 不使用3D点云
  
  -- 查找坐标变换的超时时间
  lookup_transform_timeout_sec = 0.2,
  
  -- 子地图发布周期
  submap_publish_period_sec = 0.05,
  
  -- 姿态发布周期
  pose_publish_period_sec = 5e-3,  -- 200Hz
  
  -- 轨迹发布周期
  trajectory_publish_period_sec = 30e-3,
  
  -- 测距数据采样率
  rangefinder_sampling_ratio = 1.,
  
  -- 里程计采样率（降低权重）
  odometry_sampling_ratio = 0.5,
  
  -- 固定帧姿态采样率
  fixed_frame_pose_sampling_ratio = 1.,
  
  -- IMU采样率（增加权重）
  imu_sampling_ratio = 1.,
  
  -- 路标采样率
  landmarks_sampling_ratio = 1.,
}

-- 2D SLAM 配置
MAP_BUILDER.use_trajectory_builder_2d = true

-- 轨迹构建器配置
TRAJECTORY_BUILDER_2D.min_range = 0.1            -- 激光雷达最小距离
TRAJECTORY_BUILDER_2D.max_range = 10.0           -- 激光雷达最大距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true        -- 使用IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- IMU 重力常数（非常重要！用于校准 IMU 方向）
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0  -- 使用 10 秒时间常数来估计重力方向

-- 运动过滤器 - 放宽条件，即使慢速运动也更新地图
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- 提高扫描匹配权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40

-- 子地图配置 - 更细的网格分辨率
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45  
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- -- 自适应体素过滤器
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.9
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100

-- -- 实时回环检测配置
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40

-- -- 子地图配置
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45  
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1 

-- 位姿图优化配置
POSE_GRAPH.optimize_every_n_nodes = 90

-- 回环检测参数 - 提高阈值避免对称环境中的误判
POSE_GRAPH.constraint_builder.min_score = 0.75              -- 提高到 0.75（原 0.65）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.80  -- 提高到 0.80（原 0.7）

-- 限制回环搜索范围
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- 只在 15 米内搜索回环
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3           -- 降低采样率，减少误判

-- 优化问题配置
POSE_GRAPH.optimization_problem.huber_scale = 5.0            -- Huber 损失函数尺度
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1.0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1.0

return options
