#!/usr/bin/env python3
"""
jetson_lidar.launch.py - Jetson实机雷达启动文件（修复版）
功能：
1. 启动rplidar_ros驱动（发布/scan话题）
2. 启动robot_state_publisher（发布TF树）
3. 启动joint_state_publisher（发布轮子joint状态）
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 获取包路径
    pkg_path = get_package_share_directory('robot_bringup_cpp')

    # 解析URDF
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 1. RPLidar驱动节点
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 1000000,
            'frame_id': 'laser_link',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': False
        }],
        output='screen'
    )

    # 2. Robot State Publisher（发布TF和机器人模型）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False}
        ]
    )

    # 3. Joint State Publisher（新增！发布轮子joint状态）
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': False}
        ],
        output='screen'
    )

    return LaunchDescription([
        rplidar_node,
        robot_state_publisher_node,
        joint_state_publisher_node,  # 新增
    ])
