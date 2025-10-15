include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 机器人的基础坐标系名称（frame id 与 ign gazebo 侧对齐）
  map_frame = "map",  -- 地图坐标系
  tracking_frame = "vehicle_robot/imu_link",  -- 跟踪 IMU 坐标系（在旋转中心）
  published_frame = "vehicle_robot/odom",     -- Cartographer 发布 map→odom 的变换
  odom_frame = "vehicle_robot/odom",          -- 里程计坐标系
  
  -- 是否提供里程计数据（设为 false，因为 Ignition 已经提供了 odom→chassis）
  provide_odom_frame = false,  -- 不提供额外的里程计帧
  
  -- 是否发布2d位姿
  publish_frame_projected_to_2d = true,  -- 发布2D投影位姿
  
  -- 使用位姿外推器（用于在扫描之间预测位姿）
  use_pose_extrapolator = true,  -- 启用位姿预测
  
  -- 是否使用里程计数据
  use_odometry = true,  -- 使用里程计数据
  
  -- 是否使用GPS数据
  use_nav_sat = false,  -- 不使用GPS
  
  -- 是否使用路标
  use_landmarks = false,  -- 不使用路标
  
  -- 传感器数量配置
  num_laser_scans = 1,         -- 使用1个激光雷达
  num_multi_echo_laser_scans = 0,  -- 多回波激光雷达数量
  num_subdivisions_per_laser_scan = 1,  -- 每个激光扫描的子划分数
  num_point_clouds = 0,        -- 不使用3D点云
  
  -- 查找坐标变换的超时时间
  lookup_transform_timeout_sec = 0.2,  -- TF变换查找超时（秒）
  
  -- 子地图发布周期
  submap_publish_period_sec = 0.05,  -- 子地图发布间隔（秒）
  
  -- 姿态发布周期
  pose_publish_period_sec = 5e-3,  -- 位姿发布间隔（秒），200Hz
  
  -- 轨迹发布周期
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布间隔（秒）
  
  -- 测距数据采样率
  rangefinder_sampling_ratio = 1.,  -- 激光雷达数据采样率（1.0=全采样）
  
  -- 里程计采样率（优化：降低以减少重影，平衡定位稳定性）
  odometry_sampling_ratio = 0.7,
  
  -- 固定帧姿态采样率
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定帧位姿采样率
  
  -- IMU采样率
  imu_sampling_ratio = 1.,  -- IMU数据采样率
  
  -- 路标采样率
  landmarks_sampling_ratio = 1.,  -- 路标采样率
}

-- 2D SLAM 配置
MAP_BUILDER.use_trajectory_builder_2d = true

-- 轨迹构建器配置
TRAJECTORY_BUILDER_2D.min_range = 0.1            -- 激光雷达最小有效距离（米）
TRAJECTORY_BUILDER_2D.max_range = 45.0           -- 激光雷达最大有效距离（米），优化：提高到45米以覆盖更大范围
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- 缺失数据射线长度
TRAJECTORY_BUILDER_2D.use_imu_data = true        -- 使用IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 启用在线相关扫描匹配

-- IMU 重力常数（非常重要！用于校准 IMU 方向）
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0  -- 使用 10 秒时间常数来估计重力方向

-- 运动过滤器 - 调整为匹配30Hz LiDAR频率（优化：增加阈值以过滤更多噪声，减少重影）
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.033  -- 最大时间间隔（秒）
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1  -- 最大距离阈值（米），提高以减少小幅移动的噪声
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)  -- 最大角度阈值（弧度），提高以减少旋转噪声

-- 扫描匹配权重（优化：降低权重以减少过度拟合和重影）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10  -- 平移匹配权重，降低以减少重影
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 35  -- 旋转匹配权重，降低以减少重影

-- 子地图配置 - 更细的网格分辨率（优化：减少数据点以降低密度，减少重影）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45  -- 每个子地图的范围数据点数，减少以降低密度
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 网格分辨率（米）

-- 启用自适应体素过滤器（优化：启用以减少点云噪声和重影）
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.9  -- 最大长度
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100  -- 最小点数

-- 位姿图优化配置
POSE_GRAPH.optimize_every_n_nodes = 90  -- 每多少节点优化一次

-- 回环检测参数 - 提高阈值避免对称环境中的误判（优化：提高阈值以减少重影误判）
POSE_GRAPH.constraint_builder.min_score = 0.80              -- 最小分数阈值，提高以减少误判
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85  -- 全局定位最小分数，提高以减少误判

-- 限制回环搜索范围（优化：减少搜索距离以聚焦局部，减少重影）
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- 最大约束距离（米），减少以避免远距离误判
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2           -- 采样率，降低以减少误判

-- 优化问题配置
POSE_GRAPH.optimization_problem.huber_scale = 5.0            -- Huber 损失函数尺度
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1.0  -- 里程计平移权重
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1.0  -- 里程计旋转权重

return options
