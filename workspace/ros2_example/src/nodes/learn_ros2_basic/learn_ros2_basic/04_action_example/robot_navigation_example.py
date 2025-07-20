#!/usr/bin/env python3
"""
机器人导航Action示例 - 模拟真实应用场景

这个例子模拟机器人导航过程，展示Action在实际应用中的作用：
- 客户端：发送导航目标（如：移动到坐标(10, 5)）
- 服务端：执行导航任务，发送进度反馈
- 结果：导航完成或失败

运行方法：
ros2 run learn_ros2_basic robot_navigation_example
"""

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer
from rclpy.action.client import ActionClient
from example_interfaces.action import Fibonacci
import time
import math


class RobotNavigationServer(Node):
    """
    机器人导航Action服务端
    
    模拟机器人导航过程：
    1. 接收目标坐标
    2. 规划路径
    3. 执行移动
    4. 发送进度反馈
    5. 返回结果
    """
    
    def __init__(self):
        super().__init__('robot_navigation_server')
        
        # 创建Action服务端，使用Fibonacci作为消息类型
        # 这里用order表示目标距离，sequence表示当前位置
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'robot_navigate',
            self.execute_navigation
        )
        
        # 机器人当前位置
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.get_logger().info('🤖 机器人导航服务端已启动')
        self.get_logger().info('📍 机器人当前位置: (0, 0)')
    
    def execute_navigation(self, goal_handle):
        """
        执行导航任务
        
        参数：
        - goal_handle: 包含目标坐标的句柄
        
        返回值：
        - result: 导航结果
        """
        # 获取目标距离（这里用order表示目标距离）
        target_distance = goal_handle.request.order
        
        self.get_logger().info(f'🎯 收到导航目标: 移动到距离 {target_distance} 米处')
        
        # 创建反馈对象
        feedback = Fibonacci.Feedback()
        
        # 模拟导航过程
        current_distance = 0.0
        step_size = 1.0  # 每步移动1米
        
        while current_distance < target_distance:
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('❌ 导航任务被取消')
                return Fibonacci.Result()
            
            # 更新机器人位置
            current_distance += step_size
            if current_distance > target_distance:
                current_distance = target_distance
            
            # 计算当前位置（简化：沿X轴移动）
            self.robot_x = current_distance
            
            # 发送进度反馈
            feedback.sequence = [int(current_distance)]
            goal_handle.publish_feedback(feedback)
            
            # 打印导航状态
            remaining = target_distance - current_distance
            if remaining > 0:
                self.get_logger().info(f'🚶 正在移动... 当前位置: ({self.robot_x:.1f}, {self.robot_y:.1f}) 剩余: {remaining:.1f}米')
            else:
                self.get_logger().info(f'✅ 到达目标位置: ({self.robot_x:.1f}, {self.robot_y:.1f})')
            
            # 模拟移动时间
            time.sleep(1.0)
        
        # 导航完成
        goal_handle.succeed()
        
        # 返回最终结果
        result = Fibonacci.Result()
        result.sequence = [int(target_distance)]
        
        self.get_logger().info('🎉 导航任务完成！')
        return result


class RobotNavigationClient(Node):
    """
    机器人导航Action客户端
    
    模拟用户发送导航命令并监控进度
    """
    
    def __init__(self):
        super().__init__('robot_navigation_client')
        
        # 创建Action客户端
        self._action_client = ActionClient(self, Fibonacci, 'robot_navigate')
        
        self.get_logger().info('👤 机器人导航客户端已启动')
    
    def navigate_to(self, target_distance):
        """
        发送导航目标
        
        参数：
        - target_distance: 目标距离（米）
        """
        # 等待服务端
        self._action_client.wait_for_server()
        
        # 创建目标
        goal_msg = Fibonacci.Goal()
        goal_msg.order = target_distance
        
        self.get_logger().info(f'📡 发送导航命令: 移动到 {target_distance} 米处')
        
        # 发送目标
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if goal_handle is None:
            self.get_logger().error('❌ 导航命令发送失败')
            return None
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ 导航命令被拒绝')
            return None
        
        self.get_logger().info('✅ 导航命令已接受，开始执行...')
        return goal_handle
    
    def navigation_feedback(self, feedback_msg):
        """
        导航反馈回调
        
        参数：
        - feedback_msg: 包含当前位置的反馈消息
        """
        current_pos = feedback_msg.feedback.sequence[0]
        self.get_logger().info(f'📊 客户端收到反馈: 机器人已移动到 {current_pos} 米处')
    
    def get_navigation_result(self, goal_handle):
        """
        获取导航结果
        
        参数：
        - goal_handle: 目标句柄
        """
        self.get_logger().info('⏳ 等待导航完成...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        final_distance = result.sequence[0]
        
        self.get_logger().info(f'🎯 导航完成！最终位置: {final_distance} 米')
        return result


def main(args=None):
    """
    主函数 - 演示机器人导航Action
    
    执行流程：
    1. 启动导航服务端
    2. 启动导航客户端
    3. 发送导航命令
    4. 监控导航进度
    5. 获取导航结果
    """
    rclpy.init(args=args)
    
    # 创建导航服务端
    navigation_server = RobotNavigationServer()
    
    # 创建导航客户端
    navigation_client = RobotNavigationClient()
    
    try:
        print("\n" + "="*60)
        print("🤖 机器人导航Action演示开始")
        print("="*60)
        print("演示内容：")
        print("1. 发送导航目标：移动到5米处")
        print("2. 实时监控导航进度")
        print("3. 查看反馈信息")
        print("4. 获取最终结果")
        print("="*60)
        
        # 发送导航目标：移动到5米处
        goal_handle = navigation_client.navigate_to(5)
        
        if goal_handle:
            print("\n📊 开始监控导航进度...")
            # 获取导航结果
            result = navigation_client.get_navigation_result(goal_handle)
            
            if result:
                print("\n" + "="*60)
                print("🎉 机器人导航演示完成！")
                print("="*60)
                print("这个例子展示了Action的核心价值：")
                print("✅ 发送目标：移动到指定位置")
                print("✅ 实时反馈：显示移动进度")
                print("✅ 获取结果：导航完成状态")
                print("✅ 可取消性：随时可以取消任务")
                print("="*60)
                print("Action vs Service 对比：")
                print("Service: 发送请求 → 等待 → 得到结果")
                print("Action:  发送目标 → 监控进度 → 得到结果")
                print("="*60)
        
    except KeyboardInterrupt:
        print("\n❌ 演示被用户中断")
    finally:
        # 清理资源
        navigation_server.destroy_node()
        navigation_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 