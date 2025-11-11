#!/usr/bin/env python3
"""
Jetson full patrol launch:

Optionally starts low-level serial / odom nodes, IMU+EKF+LiDAR+Nav2,
then patrol manager (looping NavigateToPose + tasks) and RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("robot_bringup_cpp")
    nav2_share = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    enable_particle_viz = LaunchConfiguration("enable_particle_viz")
    start_lowlevel = LaunchConfiguration("start_lowlevel")
    waypoints_file = LaunchConfiguration("waypoints_file")
    loop = LaunchConfiguration("loop")
    default_wait_s = LaunchConfiguration("default_wait_s")
    task_default_timeout_s = LaunchConfiguration("task_default_timeout_s")
    task_default_retry = LaunchConfiguration("task_default_retry")
    status_hz = LaunchConfiguration("status_hz")
    log_dir = LaunchConfiguration("log_dir")
    log_keep_runs = LaunchConfiguration("log_keep_runs")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock"),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(pkg_share, "maps", "office_map.yaml"),
            description="Full path to map YAML"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            description="Nav2 parameter file"),
        DeclareLaunchArgument(
            "enable_particle_viz",
            default_value="true",
            description="Enable ParticleCloud->PoseArray converter"),
        DeclareLaunchArgument(
            "start_lowlevel",
            default_value="true",
            description="Start serial/odom/imu low-level nodes"),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=os.path.join(pkg_share, "config", "patrol_waypoints.yaml"),
            description="YAML file describing patrol waypoints"),
        DeclareLaunchArgument("loop", default_value="true", description="Loop patrol route"),
        DeclareLaunchArgument(
            "default_wait_s",
            default_value="5.0",
            description="Default wait time per waypoint"),
        DeclareLaunchArgument(
            "task_default_timeout_s",
            default_value="10.0",
            description="Default task timeout seconds"),
        DeclareLaunchArgument(
            "task_default_retry",
            default_value="2",
            description="Default task retry count"),
        DeclareLaunchArgument(
            "status_hz",
            default_value="1.0",
            description="Patrol status publish frequency"),
        DeclareLaunchArgument(
            "log_dir",
            default_value="/home/nvidia/robot_ws/log/patrol",
            description="Directory for patrol logs"),
        DeclareLaunchArgument(
            "log_keep_runs",
            default_value="10",
            description="How many patrol runs of logs to retain"),
    ]

    # Low-level nodes (serial bridge + odom + imu)
    lowlevel_nodes = GroupAction(
        actions=[
            Node(
                package="robot_bringup_cpp",
                executable="serial_bridge_node",
                name="serial_bridge_node",
                output="screen",
                parameters=[{
                    "serial_port": "/dev/cmd_usb",
                    "baud_rate": 115200,
                    "use_sim_time": use_sim_time,
                }],
            ),
            Node(
                package="robot_bringup_cpp",
                executable="odom_publisher_node_cpp",
                name="odom_publisher",
                output="screen",
                parameters=[{
                    "serial_port": "/dev/encoder_usb",
                    "baud_rate": 115200,
                    "wheel_radius": 0.0645,
                    "wheel_separation": 0.23364,
                    "encoder_ppr": 4096,
                    "publish_tf": False,
                    "use_sim_time": use_sim_time,
                }],
            ),
        ],
        condition=IfCondition(start_lowlevel),
    )

    # Navigation stack (reuse existing navigation launch)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "jetson_navigation_client.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "params_file": params_file,
            "enable_particle_viz": enable_particle_viz,
            "start_rviz": "false",
        }.items(),
    )

    # Patrol manager node
    patrol_manager_node = Node(
        package="robot_bringup_cpp",
        executable="patrol_manager.py",
        name="patrol_manager",
        output="screen",
        parameters=[{
            "waypoints_file": waypoints_file,
            "loop": loop,
            "default_wait_s": default_wait_s,
            "task_default_timeout_s": task_default_timeout_s,
            "task_default_retry": task_default_retry,
            "status_hz": status_hz,
            "log_dir": log_dir,
            "log_keep_runs": log_keep_runs,
        }],
    )

    # RViz (reuse navigation configuration)
    return LaunchDescription(
        declare_args + [
            lowlevel_nodes,
            nav_launch,
            patrol_manager_node,
        ]
    )
