#!/usr/bin/env python3
"""
最简单的Action客户端示例
演示Action客户端的基本用法
"""

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from example_interfaces.action import Fibonacci


class SimpleActionClient(Node):
    """
    简单Action客户端
    """
    
    def __init__(self):
        super().__init__('simple_action_client')
        
        # 创建Action客户端
        self._action_client = ActionClient(self, Fibonacci, 'simple_count')
        
        self.get_logger().info('简单Action客户端已启动')
    
    def send_goal(self, target):
        """
        发送目标
        """
        # 等待服务端
        self._action_client.wait_for_server()
        
        # 创建目标
        goal_msg = Fibonacci.Goal()
        goal_msg.order = target
        
        # 发送目标
        self.get_logger().info(f'发送目标: 计数到 {target}')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if goal_handle is None:
            self.get_logger().error('目标发送失败')
            return None
        
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return None
        
        self.get_logger().info('目标已接受')
        return goal_handle
    
    def feedback_callback(self, feedback_msg):
        """
        反馈回调
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'收到反馈: {feedback.sequence}')
    
    def get_result(self, goal_handle):
        """
        获取结果
        """
        self.get_logger().info('等待结果...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        self.get_logger().info(f'得到结果: {result.sequence}')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    client = SimpleActionClient()
    
    # 发送目标：计数到5
    goal_handle = client.send_goal(5)
    
    if goal_handle:
        # 获取结果
        result = client.get_result(goal_handle)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 