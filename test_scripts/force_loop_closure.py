#!/usr/bin/env python3
"""
force_loop_closure.py - 尝试强制触发slam_toolbox回环检测

使用方法：
    ros2 run slam_toolbox async_slam_toolbox_node 运行时
    python3 force_loop_closure.py
"""

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import LoopClosure
import time


class ForceLoopClosure(Node):
    def __init__(self):
        super().__init__('force_loop_closure')
        self.get_logger().info('🔧 强制回环检测节点已启动')
        
        # 创建服务客户端
        self.loop_closure_client = self.create_client(
            LoopClosure, 
            '/slam_toolbox/manual_loop_closure'
        )
        
        # 等待服务可用
        self.get_logger().info('⏳ 等待slam_toolbox服务...')
        while not self.loop_closure_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('   服务未就绪，继续等待...')
        
        self.get_logger().info('✅ 服务已就绪！')
    
    def trigger_loop_closure(self):
        """触发手动回环检测"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 尝试触发回环检测...')
        self.get_logger().info('=' * 60)
        
        request = LoopClosure.Request()
        
        future = self.loop_closure_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            self.get_logger().info('✅ 回环检测请求已发送')
            self.get_logger().info('')
            self.get_logger().info('⏳ 等待5秒，查看slam_toolbox是否触发优化...')
            time.sleep(5)
            self.get_logger().info('')
            self.get_logger().info('请查看主终端的slam_toolbox输出：')
            self.get_logger().info('  - 如果看到"Loop closure"相关信息 → 成功！')
            self.get_logger().info('  - 如果还是"No changes to apply" → 失败')
        else:
            self.get_logger().error('❌ 回环检测请求失败')


def main():
    rclpy.init()
    
    print('')
    print('=' * 60)
    print('🔧 slam_toolbox 强制回环检测工具')
    print('=' * 60)
    print('')
    print('说明：')
    print('  此工具会尝试调用slam_toolbox的手动回环服务')
    print('  但是否成功取决于当前扫描数据的相似度')
    print('')
    print('如果失败，建议：')
    print('  1. 接受当前结果（-1.20°已经很好）')
    print('  2. 保存地图继续使用')
    print('  3. 或重启launch文件应用新配置')
    print('')
    print('=' * 60)
    print('')
    
    node = ForceLoopClosure()
    
    try:
        # 触发回环检测
        node.trigger_loop_closure()
        
        print('')
        print('=' * 60)
        print('🏁 任务完成')
        print('=' * 60)
        print('')
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

