#!/usr/bin/env python3
"""
IMU角度实时监控工具
功能：实时显示JY901B IMU的姿态角（Roll、Pitch、Yaw）
用法：python3 imu_angle_monitor.py
     或: ./imu_angle_monitor.py (需要先chmod +x)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math


class IMUAngleMonitor(Node):
    def __init__(self):
        super().__init__('imu_angle_monitor')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)
        
        print("\n" + "="*70)
        print("🧭 IMU角度实时监控工具")
        print("="*70)
        print("功能：实时显示JY901B IMU的姿态角")
        print()
        print("角度说明：")
        print("  • Yaw（航向）  : 车子水平旋转角度")
        print("    - 顺时针转  → 负值增大（如 -90°）")
        print("    - 逆时针转  → 正值增大（如 +90°）")
        print("  • Roll（横滚） : 车子左右倾斜角度")
        print("  • Pitch（俯仰）: 车子前后倾斜角度")
        print()
        print("按 Ctrl+C 停止监控")
        print("="*70)
        print()
        
    def imu_callback(self, msg):
        """处理IMU消息并显示角度"""
        # 提取四元数
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        
        # 四元数转欧拉角
        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)
        
        # 转换为角度
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # 实时显示（覆盖同一行）
        print(f"\r📐 Yaw: {yaw_deg:+7.2f}° | Roll: {roll_deg:+6.2f}° | Pitch: {pitch_deg:+6.2f}°  ", 
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
    monitor = IMUAngleMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n✅ 监控已停止")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

