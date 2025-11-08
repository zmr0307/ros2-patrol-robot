#!/usr/bin/env python3
"""
SLAM建图实时监控工具
功能：
1. 显示map→odom的角度偏移和距离
2. 显示EKF、编码器、IMU的yaw角度对比
3. 实时更新，方便调试建图精度

用法：
  python3 slam_monitor.py
  或: ./slam_monitor.py (需要先chmod +x)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
import math
import sys


class SLAMMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅各数据源
        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_callback, 10)
        self.encoder_sub = self.create_subscription(
            Odometry, '/odom', self.encoder_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        
        # 数据存储
        self.ekf_yaw = 0.0
        self.encoder_yaw = 0.0
        self.imu_yaw = 0.0
        self.map_odom_yaw = 0.0
        self.map_odom_distance = 0.0
        self.robot_yaw = 0.0
        self.robot_distance = 0.0
        self.robot_x = 0.0  # 机器人X轴位置
        self.robot_y = 0.0  # 机器人Y轴位置
        
        # 定时器：10Hz刷新显示
        self.timer = self.create_timer(0.1, self.display_callback)
        
        # 打印标题
        self.print_header()
        
    def print_header(self):
        """打印表头"""
        print("\n" + "="*100)
        print("🗺️  SLAM建图实时监控")
        print("="*100)
        print("\n【坐标系变换】")
        print("  • map → odom:")
        print("    - 角度偏移：地图坐标系相对于里程计坐标系的旋转角度")
        print("    - 平移距离：两个坐标系原点之间的距离")
        print("  • 机器人位置 (map → base_footprint):")
        print("    - 距离起点：机器人底盘（地面投影）距离地图原点的xy平面距离")
        print("    - 朝向角度：机器人在地图中的朝向")
        print("\n【角度对比】")
        print("  - EKF：融合后的角度（编码器+IMU）")
        print("  - 编码器：仅轮式里程计的角度")
        print("  - IMU：仅陀螺仪的角度")
        print("\n按 Ctrl+C 停止监控")
        print("="*100)
        print()
    
    def ekf_callback(self, msg):
        """EKF融合数据回调"""
        self.ekf_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
    
    def encoder_callback(self, msg):
        """编码器数据回调"""
        self.encoder_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        self.imu_yaw = self.quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
    
    def display_callback(self):
        """定时显示数据"""
        # 获取map→odom变换
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
            
            # 提取变换数据
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # 计算角度偏移
            self.map_odom_yaw = self.quaternion_to_yaw(rot.x, rot.y, rot.z, rot.w)
            
            # 计算平移距离
            self.map_odom_distance = math.sqrt(trans.x**2 + trans.y**2)
            
        except Exception as e:
            # map→odom变换不存在（建图未开始）
            self.map_odom_yaw = 0.0
            self.map_odom_distance = 0.0
        
        # 获取机器人位置（map→base_footprint变换）
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            
            # 提取机器人位置数据
            robot_trans = robot_transform.transform.translation
            robot_rot = robot_transform.transform.rotation
            
            # 计算机器人朝向角度
            self.robot_yaw = self.quaternion_to_yaw(
                robot_rot.x, robot_rot.y, robot_rot.z, robot_rot.w)
            
            # 保存X和Y轴位置
            self.robot_x = robot_trans.x
            self.robot_y = robot_trans.y
            
            # 计算机器人距离地图原点的距离（仅xy平面，不含高度）
            self.robot_distance = math.sqrt(
                robot_trans.x**2 + robot_trans.y**2)
            
        except Exception as e:
            # map→base_footprint变换不存在
            self.robot_yaw = 0.0
            self.robot_distance = 0.0
            self.robot_x = 0.0
            self.robot_y = 0.0
        
        # 计算角度差值
        ekf_encoder_diff = self.normalize_angle(self.ekf_yaw - self.encoder_yaw)
        ekf_imu_diff = self.normalize_angle(self.ekf_yaw - self.imu_yaw)
        encoder_imu_diff = self.normalize_angle(self.encoder_yaw - self.imu_yaw)
        
        # 实时显示（覆盖三行）
        # 第一行：坐标系变换信息
        line1 = (
            f"🗺️ map→odom: 角度 {self.map_odom_yaw:+7.2f}° | 距离 {self.map_odom_distance:6.3f}m  │  "
            f"🤖 机器人: 距起点 {self.robot_distance:6.3f}m | 朝向 {self.robot_yaw:+7.2f}°"
        )
        # 第二行：机器人XY坐标
        line2 = (
            f"📍 机器人坐标: X = {self.robot_x:+7.3f}m | Y = {self.robot_y:+7.3f}m"
        )
        # 第三行：传感器角度对比
        line3 = (
            f"📊 EKF: {self.ekf_yaw:+7.2f}° | "
            f"🔧 编码器: {self.encoder_yaw:+7.2f}° | "
            f"🧭 IMU: {self.imu_yaw:+7.2f}°  │  "
            f"Δ(EKF-编码器): {ekf_encoder_diff:+6.2f}° | "
            f"Δ(EKF-IMU): {ekf_imu_diff:+6.2f}°"
        )
        
        # 清除当前行并显示三行
        print(f"\r{line1}\n{line2}\n{line3}", end='\r\033[2A', flush=True)
    
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """四元数转yaw角（度）"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)
    
    @staticmethod
    def normalize_angle(angle):
        """归一化角度到[-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle


def main():
    """主函数"""
    rclpy.init()
    monitor = SLAMMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n" + "="*100)
        print("✅ 监控已停止")
        print("="*100)
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

