#!/usr/bin/env python3

"""
Cartographer SLAM Launch File
启动 Cartographer 进行 2D SLAM 建图
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的路径
    pkg_share = get_package_share_directory("robo_ctrl")

    # Cartographer 配置文件路径
    cartographer_config_dir = os.path.join(pkg_share, "config")
    configuration_basename = "cartographer_2d.lua"

    # Launch 参数
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    resolution = LaunchConfiguration(
        "resolution", default="0.1"
    )  # 从 0.05 改为 0.1，减少地图大小
    publish_period_sec = LaunchConfiguration(
        "publish_period_sec", default="0.05"
    )  # 20 Hz 更新

    # Cartographer 节点
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            cartographer_config_dir,
            "-configuration_basename",
            configuration_basename,
        ],
        remappings=[
            ("scan", "/laser_scan"),  # 激光雷达话题
            ("odom", "/odom"),  # 里程计话题
            ("imu", "/imu"),  # IMU话题
        ],
    )

    # 占据栅格地图节点（将子地图转换为ROS占据栅格地图）
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
        arguments=["-resolution", "0.1", "-publish_period_sec", "0.05"],
    )

    return LaunchDescription(
        [
            # 声明 Launch 参数
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.05",
                description="Resolution of a grid cell in the published occupancy grid",
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value="0.1",
                description="OccupancyGrid publishing period",
            ),
            # 启动节点
            cartographer_node,
            occupancy_grid_node,
        ]
    )
