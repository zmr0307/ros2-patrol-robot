#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Get package path
    pkg_path = get_package_share_directory('robot_bringup_cpp')

    # Serial bridge node (speed command sender)
    serial_bridge_node = Node(
        package='robot_bringup_cpp',
        executable='serial_bridge_node',
        name='serial_bridge',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyCH341USB0'},
            {'baud_rate': 115200},
            {'use_sim_time': False}
        ]
    )

    # Odometry publisher node (receive from STM32)
    # publish_tf: False - EKF will handle TF broadcasting
    odom_publisher_node = Node(
        package='robot_bringup_cpp',
        executable='odom_publisher_node_cpp',
        name='odom_publisher',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyCH341USB1'},
            {'baud_rate': 115200},
            {'wheel_radius': 0.064},  # 图纸规格直径128mm
            {'wheel_separation': 0.228},   # 360°标定22.8cm（348.4°→360°，修正-11.6°）
            {'encoder_ppr': 4096},  # 1024线 × 4倍频
            {'base_frame_id': 'base_footprint'},
            {'odom_frame_id': 'odom'},
            {'publish_tf': False},  # EKF接管TF发布
            {'use_sim_time': False}
        ]
    )

    # IMU publisher node (JY901B IMU sensor)
    imu_node = Node(
        package='robot_bringup_cpp',
        executable='imu_publisher_node',
        name='imu_publisher',
        output='screen',
        parameters=[
            {'serial_port': '/dev/imu_usb'},
            {'baud_rate': 115200},
            {'frame_id': 'imu_link'},
            {'publish_rate': 50.0},
            {'use_sim_time': False}
        ]
    )

    # EKF localization node (fuses encoder + IMU)
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf_localization.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',  # ✅ 与ekf_localization.yaml第1行一致
        output='screen',
        parameters=[ekf_config_path]
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

    # Include lidar launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'jetson_lidar.launch.py')
        )
    )

    # SLAM Toolbox config path
    slam_config_path = os.path.join(pkg_path, 'config', 'slam_toolbox_real.yaml')

    # SLAM Toolbox node (带详细日志输出)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path],
        arguments=['--ros-args', '--log-level', 'slam_toolbox:=DEBUG']  # DEBUG级别查看详细信息
    )

    # Startup information
    start_info = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            'Jetson Real Robot SLAM System (with IMU + EKF Fusion)\n',
            '=' * 80, '\n',
            'Sensor Fusion Pipeline:\n',
            '   1. Encoder Odometry (/odom) - Position + Linear Velocity\n',
            '   2. IMU (/imu/data_raw) - Angular Velocity (Z-axis)\n',
            '   3. EKF Fusion (/odometry/filtered) - Corrected Pose\n',
            '   4. TF: EKF publishes odom -> base_footprint\n',
            '=' * 80, '\n',
            'Hardware Connections:\n',
            '   Speed Command: Jetson -> /dev/ttyCH341USB0 -> STM32 USART1\n',
            '   Odometry: STM32 USART2 -> /dev/ttyCH341USB1 -> Jetson\n',
            '   IMU (JY901B): Jetson -> /dev/imu_usb (115200 baud, 50Hz)\n',
            '   Lidar: RPLidar -> /dev/ttyUSB0 -> Jetson\n',
            '=' * 80, '\n',
            'Active Nodes:\n',
            '   ✓ serial_bridge_node (Speed Command Sender)\n',
            '   ✓ odom_publisher_node (Encoder Odometry, TF: OFF)\n',
            '   ✓ imu_publisher_node (JY901B IMU Data)\n',
            '   ✓ ekf_localization_node (Sensor Fusion, TF: ON)\n',
            '   ✓ rplidar_node (Laser Scan)\n',
            '   ✓ slam_toolbox (Mapping)\n',
            '=' * 80, '\n',
            'Control Methods:\n',
            '   Method 1 (Keyboard):\n',
            '      ros2 run teleop_twist_keyboard teleop_twist_keyboard\n',
            '   Method 2 (Direct Publish):\n',
            '      ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\\n',
            '        "{linear: {x: 0.05}, angular: {z: 0.0}}" --once\n',
            '=' * 80, '\n',
            'Monitoring Commands:\n',
            '   ros2 topic hz /odom                  # Encoder odometry\n',
            '   ros2 topic hz /imu/data_raw          # IMU data\n',
            '   ros2 topic hz /odometry/filtered     # EKF output\n',
            '   ros2 topic echo /odometry/filtered   # View fused pose\n',
            '   ros2 run tf2_ros view_frames         # Check TF tree\n',
            '   ros2 run tf2_ros tf2_echo odom base_footprint  # Check TF transform\n',
            '=' * 80, '\n',
            'Important Notes:\n',
            '   1. Serial permissions: sudo chmod 666 /dev/ttyCH341USB* /dev/imu_usb\n',
            '   2. IMU calibration: 6-axis algorithm, 115200 baud, 50Hz output\n',
            '   3. EKF fuses: Encoder (x,y,vx) + IMU (angular_velocity.z)\n',
            '   4. Expected rotation drift: ≤5° after 3×360° turns\n',
            '   5. SLAM uses: EKF output for accurate pose estimation\n',
            '=' * 80, '\n'
        ]
    )

    return LaunchDescription([
        start_info,
        # serial_bridge_node,    # Speed command sender - LAUNCH SEPARATELY IN TERMINAL 1
        # odom_publisher_node,   # Encoder odometry (TF: OFF) - LAUNCH SEPARATELY IN TERMINAL 2
        imu_node,              # IMU data publisher
        imu_tf_node,           # Static TF: base_footprint -> imu_link
        ekf_node,              # EKF sensor fusion (TF: ON)
        lidar_launch,          # Lidar + TF
        slam_toolbox_node,     # SLAM mapping
    ])
