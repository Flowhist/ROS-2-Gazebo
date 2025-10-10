#!/usr/bin/env python3

"""
完整的仿真+SLAM+可视化启动文件
同时启动 Ignition Gazebo 仿真、Cartographer SLAM 和 RViz2 可视化
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的路径
    pkg_share = get_package_share_directory("robo_ctrl")

    # RViz 配置文件路径
    rviz_config_file = os.path.join(pkg_share, "config", "cartographer.rviz")

    # 包含仿真启动文件
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "simulation.launch.py")
        )
    )

    # 包含 Cartographer 启动文件
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "cartographer.launch.py")
        )
    )

    # RViz2 节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            simulation_launch,
            cartographer_launch,
            rviz_node,
        ]
    )
