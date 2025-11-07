#!/usr/bin/env python3
"""
patrol_navigation.py - 自动巡检导航节点

功能：
1. 自动设置初始位姿到map坐标系原点(0, 0, 0°)
2. 按预设路径点循环巡检
3. 实时监控导航状态
4. 支持循环巡检和单次巡检模式

使用方法：
  ros2 run robot_bringup_cpp patrol_navigation.py

作者：robot_bringup_cpp项目
日期：2025-11-07
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import time
import math


class PatrolNavigation(Node):
    def __init__(self):
        super().__init__('patrol_navigation')
        
        # 声明参数
        self.declare_parameter('loop_mode', True)  # 是否循环巡检
        self.declare_parameter('wait_time', 3.0)   # 到达路径点后的等待时间（秒）
        
        self.loop_mode = self.get_parameter('loop_mode').value
        self.wait_time = self.get_parameter('wait_time').value
        
        # 初始位姿发布器（用于设置AMCL初始位姿）
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Nav2导航动作客户端
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # 巡检路径点（map坐标系）
        # 格式：[(x, y, yaw_degrees), ...]
        # 用户自定义路径：直走→左转45°→前进1m→回起点
        self.waypoints = [
            (0.0, 0.0, 0.0),      # 起点：头朝前（0°）
            (0.5, 0.0, 0.0),      # 点1：直走50cm，保持朝向
            (0.5, 0.0, 45.0),     # 点2：原地左转45°
            (1.207, 0.707, 45.0), # 点3：沿45°方向前进1米
            (0.0, 0.0, 0.0),      # 点4：回到起点，头朝前
        ]
        
        # ============================================================
        # 💡 如何自定义巡检路径？
        # ============================================================
        # 方法1：在RViz中查看地图，找到可通行区域的坐标
        #   1. 打开RViz，显示地图
        #   2. 用鼠标点击地图上的点，在底部显示坐标
        #   3. 记录安全的坐标点
        #   4. 修改上面的self.waypoints列表
        #
        # 方法2：让机器人先手动导航到关键点
        #   1. 在RViz中使用"2D Nav Goal"手动导航
        #   2. 记录每个点的坐标（运行：ros2 topic echo /amcl_pose）
        #   3. 修改上面的self.waypoints列表
        #
        # 方法3：使用参数文件（高级）
        #   创建一个YAML文件存储路径点，然后从参数加载
        # ============================================================
        
        self.current_waypoint_index = 0
        self.navigation_active = False
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🤖 自动巡检导航节点已启动')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'循环模式: {"开启" if self.loop_mode else "关闭"}')
        self.get_logger().info(f'路径点等待时间: {self.wait_time}秒')
        self.get_logger().info(f'巡检路径点数量: {len(self.waypoints)}')
        self.get_logger().info('=' * 70)
        
        # 等待Nav2准备就绪
        self.get_logger().info('⏳ 等待Nav2导航系统就绪...')
        self.wait_for_nav2()
        
        # 自动设置初始位姿
        self.get_logger().info('📍 设置初始位姿到map坐标系原点...')
        self.set_initial_pose()
        
        # 等待AMCL初始化
        time.sleep(2.0)
        
        # 开始巡检
        self.get_logger().info('🚀 开始自动巡检！')
        self.start_patrol()
    
    def wait_for_nav2(self):
        """等待Nav2导航系统准备就绪"""
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('   Nav2尚未就绪，继续等待...')
        self.get_logger().info('✅ Nav2导航系统已就绪！')
    
    def set_initial_pose(self):
        """自动设置初始位姿到map坐标系原点"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 位置：map坐标系原点
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        
        # 姿态：0度（四元数）
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # 协方差（表示初始位姿的不确定性）
        initial_pose.pose.covariance = [0.0] * 36
        initial_pose.pose.covariance[0] = 0.25   # x方差
        initial_pose.pose.covariance[7] = 0.25   # y方差
        initial_pose.pose.covariance[35] = 0.06  # yaw方差
        
        # 发布初始位姿
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info('✅ 初始位姿已设置：(0.0, 0.0, 0.0°)')
    
    def start_patrol(self):
        """开始巡检"""
        self.navigate_to_next_waypoint()
    
    def navigate_to_next_waypoint(self):
        """导航到下一个路径点"""
        if self.current_waypoint_index >= len(self.waypoints):
            if self.loop_mode:
                # 循环模式：回到第一个路径点
                self.current_waypoint_index = 0
                self.get_logger().info('🔄 完成一轮巡检，重新开始...')
            else:
                # 单次模式：停止巡检
                self.get_logger().info('🎉 巡检完成！')
                return
        
        # 获取当前路径点
        x, y, yaw_deg = self.waypoints[self.current_waypoint_index]
        yaw_rad = yaw_deg * math.pi / 180.0
        
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'📍 导航到路径点 {self.current_waypoint_index + 1}/{len(self.waypoints)}')
        self.get_logger().info(f'   位置: ({x:.2f}, {y:.2f}) m')
        self.get_logger().info(f'   朝向: {yaw_deg:.1f}°')
        self.get_logger().info('=' * 70)
        
        # 创建目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # 角度转四元数
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        
        # 创建导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # 发送导航目标
        self.navigation_active = True
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)
    
    def navigation_goal_response_callback(self, future):
        """导航目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 导航目标被拒绝！')
            self.navigation_active = False
            return
        
        self.get_logger().info('✅ 导航目标已接受，开始导航...')
        
        # 等待导航结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """导航反馈回调（可选：显示导航进度）"""
        feedback = feedback_msg.feedback
        # 可以在这里添加进度显示逻辑
        pass
    
    def navigation_result_callback(self, future):
        """导航结果回调"""
        result = future.result().result
        status = future.result().status
        
        self.navigation_active = False
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ 成功到达路径点！')
            
            # 等待指定时间
            if self.wait_time > 0:
                self.get_logger().info(f'⏳ 等待 {self.wait_time} 秒...')
                time.sleep(self.wait_time)
            
            # 前往下一个路径点
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
            
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('❌ 导航失败：目标被中止')
            self.get_logger().info('🔄 重试当前路径点...')
            time.sleep(2.0)
            self.navigate_to_next_waypoint()
            
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('⚠️  导航被取消')
            
        else:
            self.get_logger().error(f'❌ 导航失败：状态码 {status}')
            self.get_logger().info('🔄 重试当前路径点...')
            time.sleep(2.0)
            self.navigate_to_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        patrol_node = PatrolNavigation()
        rclpy.spin(patrol_node)
    except KeyboardInterrupt:
        print('\n⏹️  巡检被用户中断')
    except Exception as e:
        print(f'❌ 错误: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

