#!/usr/bin/env python3
"""
最简单的Action客户端示例
演示Action客户端的基本用法

Action接口定义 (example_interfaces.action.Fibonacci):
# Goal（目标）
int32 order          # 要计算的斐波那契数列长度
---
# Result（结果）
int32[] sequence     # 完整的斐波那契数列 [0,1,1,2,3,5,8,13,...]
---
# Feedback（反馈）
int32[] sequence     # 当前已计算的斐波那契数列 [0,1,1,2,3]
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
        
        参数：
        - target: 要计算的斐波那契数列长度，对应 Fibonacci.Goal.order
        
        返回值：
        - goal_handle: 目标句柄，用于后续获取结果
        """
        # 等待服务端
        self._action_client.wait_for_server()
        
        # 创建目标 (Fibonacci.Goal)
        goal_msg = Fibonacci.Goal()
        goal_msg.order = target  # 设置要计算的斐波那契数列长度
        
        # 发送目标
        self.get_logger().info(f'发送目标: 计算 {target} 个斐波那契数')
        
        # 异步发送目标，并设置反馈回调
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # 等待目标发送完成
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
        
        参数：
        - feedback_msg: 包含 Fibonacci.Feedback 的反馈消息
        
        反馈内容：
        - feedback.sequence: 当前已计算的斐波那契数列 [0,1,1,2,3]
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'收到反馈: {feedback.sequence}')
    
    def get_result(self, goal_handle):
        """
        获取结果
        
        参数：
        - goal_handle: 目标句柄
        
        返回值：
        - result: Fibonacci.Result 对象
          - result.sequence: 完整的斐波那契数列 [0,1,1,2,3,5,8,13,...]
        """
        # 第1行：打印日志，告诉用户正在等待结果
        self.get_logger().info('等待结果...')
        
        # 第2行：异步请求获取结果，返回一个Future对象
        # Future对象代表一个"将来会完成的操作"
        get_result_future = goal_handle.get_result_async()
        
        # 第3行：等待Future完成，阻塞当前线程直到结果可用
        # 这行代码会一直等待，直到Action服务端完成计算并返回结果
        rclpy.spin_until_future_complete(self, get_result_future)
        
        # 第4行：从Future中提取实际的结果数据
        # get_result_future.result() 返回一个包含result字段的对象
        # .result 获取其中的Fibonacci.Result对象
        result = get_result_future.result().result
        
        # 第5行：打印获取到的斐波那契数列
        self.get_logger().info(f'得到结果: {result.sequence}')
        
        # 第6行：返回结果对象，供调用者使用
        return result


def main(args=None):
    rclpy.init(args=args)
    client = SimpleActionClient()
    
    # 发送目标：计算20个斐波那契数
    goal_handle = client.send_goal(20)
    
    if goal_handle:
        # 获取结果
        result = client.get_result(goal_handle)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 