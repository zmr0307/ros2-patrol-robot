#!/usr/bin/env python3
"""
编码器里程计 vs EKF融合角度对比监控工具
功能：实时对比 /odom (编码器) 和 /odometry/filtered (EKF) 的yaw角度
用法：python3 odom_ekf_compare.py
     或: ./odom_ekf_compare.py (需要先chmod +x)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class OdomEKFCompare(Node):
    def __init__(self):
        super().__init__('odom_ekf_compare')
        
        # 订阅编码器里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # 订阅EKF融合输出
        self.ekf_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.ekf_callback,
            10)
        
        # 存储角度
        self.odom_yaw = None
        self.ekf_yaw = None
        
        print("\n" + "="*80)
        print("📊 编码器里程计 vs EKF融合角度对比工具")
        print("="*80)
        print("功能：实时对比编码器和EKF的yaw角度差异")
        print()
        print("数据来源：")
        print("  • 编码器里程计: /odom (wheel encoders)")
        print("  • EKF融合输出 : /odometry/filtered (encoders + IMU angular velocity)")
        print()
        print("差值说明：")
        print("  • 差值 < 3°  : 正常 ✅")
        print("  • 差值 3-5° : 注意 ⚠️")
        print("  • 差值 > 5°  : 异常 ❌ (可能存在滞后或配置问题)")
        print()
        print("按 Ctrl+C 停止监控")
        print("="*80)
        print()
        
    def odom_callback(self, msg):
        """处理编码器里程计消息"""
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        _, _, yaw = self.quaternion_to_euler(x, y, z, w)
        self.odom_yaw = math.degrees(yaw)
        
        self.display_comparison()
    
    def ekf_callback(self, msg):
        """处理EKF融合输出消息"""
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        _, _, yaw = self.quaternion_to_euler(x, y, z, w)
        self.ekf_yaw = math.degrees(yaw)
        
        self.display_comparison()
    
    def display_comparison(self):
        """实时显示对比结果"""
        if self.odom_yaw is None or self.ekf_yaw is None:
            return
        
        # 计算差值（考虑角度循环：-180~180）
        diff = self.ekf_yaw - self.odom_yaw
        
        # 归一化到[-180, 180]
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        
        # 根据差值大小选择状态符号
        if abs(diff) < 3:
            status = "✅"
            color = ""
        elif abs(diff) < 5:
            status = "⚠️ "
            color = ""
        else:
            status = "❌"
            color = ""
        
        # 实时显示（覆盖同一行）
        print(f"\r{status} 编码器: {self.odom_yaw:+7.2f}° | EKF: {self.ekf_yaw:+7.2f}° | 差值: {diff:+6.2f}°  ", 
              end='', flush=True)
    
    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        将四元数转换为欧拉角（Roll、Pitch、Yaw）
        
        参数:
            x, y, z, w: 四元数分量
        
        返回:
            (roll, pitch, yaw): 欧拉角（弧度）
        """
        # Roll (x轴旋转)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y轴旋转)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z轴旋转)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


def main():
    """主函数"""
    rclpy.init()
    monitor = OdomEKFCompare()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n✅ 监控已停止")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

