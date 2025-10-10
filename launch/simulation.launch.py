#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径，用于rviz中显示robo模型
    pkg_share = get_package_share_directory("robo_ctrl")
    urdf_file = os.path.join(pkg_share, "models", "vehicle_robot", "robot.urdf")
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    # 获取world.sdf文件路径
    world_file = "/home/flowhist/ign_ws/src/robo_ctrl/models/world.sdf"

    # 启动Ignition Gazebo
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", world_file, "-r"], output="screen", shell=False
    )
    # Robot State Publisher - 发布机器人模型描述
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_desc,
                "publish_frequency": 20.0,
            }
        ],
    )

    # ROS-Gazebo桥接 - Joint State
    bridge_joint_state = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/car_world/model/vehicle_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        remappings=[
            ("/world/car_world/model/vehicle_robot/joint_state", "/joint_states")
        ],
        output="screen",
    )

    # ROS-Gazebo桥接 - CMD_VEL
    bridge_cmd_vel = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"],
        output="screen",
    )

    # ROS-Gazebo桥接 - Lidar (重映射到/laser_scan)
    bridge_lidar = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"],
        remappings=[("/lidar", "/laser_scan")],
        output="screen",
    )

    # ROS-Gazebo桥接 - IMU
    bridge_imu = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
        output="screen",
    )

    # ROS-Gazebo桥接 - Odometry
    bridge_odom = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/vehicle_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"
        ],
        remappings=[("/model/vehicle_robot/odometry", "/odom")],
        output="screen",
    )

    # ROS-Gazebo桥接 - TF
    bridge_tf = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/vehicle_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
        ],
        remappings=[("/model/vehicle_robot/tf", "/tf")],
        output="screen",
    )

    # ROS-Gazebo桥接 - Clock
    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            bridge_joint_state,
            bridge_cmd_vel,
            bridge_lidar,
            bridge_imu,
            bridge_odom,
            bridge_tf,
            bridge_clock,
        ]
    )
