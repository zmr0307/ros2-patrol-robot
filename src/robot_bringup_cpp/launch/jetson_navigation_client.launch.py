#!/usr/bin/env python3
"""
jetson_navigation_client.launch.py - Jetson端Nav2自主导航（All-in-One）
功能：
1. 启动IMU + EKF传感器融合（输出 /odometry/filtered）
2. 启动LiDAR + TF（通过jetson_lidar.launch.py）
3. 启动Nav2导航堆栈（AMCL定位 + 路径规划 + 运动控制）
4. 启动RViz2可视化 + AMCL粒子云转换

注意：需要先启动底层节点（2个终端）：
  终端1: ros2 run robot_bringup_cpp serial_bridge_node
  终端2: ros2 run robot_bringup_cpp odom_publisher_node_cpp
  终端3: 本launch文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_bringup_pkg = get_package_share_directory('robot_bringup_cpp')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for real robot)'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(robot_bringup_pkg, 'maps', 'office_map.yaml'),
        description='Full path to the map yaml file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robot_bringup_pkg, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters'
    )

    # 可选：启用AMCL粒子云可视化转换节点
    enable_particle_viz_arg = DeclareLaunchArgument(
        'enable_particle_viz',
        default_value='true',
        description='Enable ParticleCloud->PoseArray converter node'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    enable_particle_viz = LaunchConfiguration('enable_particle_viz')

    # ========== IMU + EKF传感器融合节点（导航需要） ==========
    # IMU publisher node (JY901B IMU sensor)
    imu_node = Node(
        package='robot_bringup_cpp',
        executable='imu_publisher_node',
        name='imu_publisher',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyCH341USB1'},  # 修正：IMU数据接收（USB1，0x55帧头）
            {'baud_rate': 115200},
            {'frame_id': 'imu_link'},
            {'publish_rate': 50.0},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Static TF: base_footprint -> imu_link
    # IMU is mounted slightly forward from center, 35cm above base_footprint
    # x=0.1m (10cm forward), y=0 (centered), z=0.35m (35cm height)
    imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        arguments=['0.1', '0', '0.35', '0', '0', '0', 'base_footprint', 'imu_link']
    )

    # EKF localization node (fuses encoder + IMU)
    ekf_config_path = os.path.join(robot_bringup_pkg, 'config', 'ekf_localization.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',  # ✅ 与ekf_localization.yaml第1行一致
        output='screen',
        parameters=[ekf_config_path]
    )
    # ================================================================

    # ========== 包含LiDAR launch（已包含robot_state_publisher）==========
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_pkg, 'launch', 'jetson_lidar.launch.py')
        )
    )
    # ====================================================================

    default_bt_xml_filename = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'default_bt_xml_filename': default_bt_xml_filename
        }.items()
    )

    rviz_config_file = os.path.join(robot_bringup_pkg, 'rviz', 'navigation.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 使用TimerAction延迟启动RViz（等待其他节点就绪）
    rviz_delayed = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )

    # 粒子云转换节点（将nav2_msgs/ParticleCloud转为geometry_msgs/PoseArray，便于RViz显示）
    particle_converter_node = Node(
        package='robot_bringup_cpp',
        executable='particle_cloud_to_pose_array.py',
        name='particle_cloud_to_pose_array',
        output='screen',
        arguments=['--in', '/particle_cloud', '--out', '/amcl/particles', '--frame', 'map'],
        condition=IfCondition(enable_particle_viz)
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        enable_particle_viz_arg,
        imu_node,              # ← IMU数据发布
        imu_tf_node,           # ← 静态TF: base_footprint -> imu_link
        ekf_node,              # ← EKF传感器融合 (输出 /odometry/filtered)
        lidar_launch,          # ← 包含LiDAR + robot_state_publisher + joint_state_publisher
        nav2_bringup_launch,   # ← Nav2导航堆栈 (使用 /odometry/filtered)
        rviz_delayed,          # ← 3秒后启动RViz
        particle_converter_node  # ← AMCL粒子云可视化转换
    ])


if __name__ == '__main__':
    generate_launch_description()
