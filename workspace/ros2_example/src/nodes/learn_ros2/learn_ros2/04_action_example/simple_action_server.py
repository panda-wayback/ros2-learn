#!/usr/bin/env python3
"""
ROS 2 Action服务端示例 - 斐波那契数列计算

功能说明：
- 接收客户端发送的目标长度（如：6）
- 计算斐波那契数列：0,1,1,2,3,5,8,13,...
- 每0.5秒发送一次进度反馈
- 完成后返回完整的斐波那契数列

使用场景：
- 学习ROS 2 Action的基本用法
- 演示长时间任务的执行过程
- 展示反馈机制的使用
- 演示真正的斐波那契数列计算

运行方法：
ros2 run learn_ros2 simple_action_server

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
from rclpy.action.server import ActionServer
from example_interfaces.action import Fibonacci
import time


class SimpleActionServer(Node):
    """
    简单Action服务端 - 斐波那契数列计算器
    
    功能：
    - 接收计算目标（如：计算6个斐波那契数）
    - 执行斐波那契数列计算（0,1,1,2,3,5）
    - 发送进度反馈
    - 返回最终结果
    """
    
    def __init__(self):
        super().__init__('simple_action_server')
        
        # 创建Action服务端
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'simple_count',
            self.execute_callback
        )
        
        self.get_logger().info('简单Action服务端已启动')
    
    def execute_callback(self, goal_handle):
        """
        Action执行回调函数 - 核心斐波那契数列计算逻辑
        
        参数：
        - goal_handle: 目标句柄，包含客户端发送的目标信息
          - goal_handle.request.order: 要计算的斐波那契数列长度（Fibonacci.Goal.order）
        
        返回值：
        - result: 包含完整斐波那契数列的结果对象（Fibonacci.Result）
          - result.sequence: 完整的斐波那契数列 [0,1,1,2,3,5,8,13,...]
        
        执行流程：
        1. 获取目标长度 (goal_handle.request.order)
        2. 计算斐波那契数列
        3. 每步发送进度反馈 (feedback.sequence)
        4. 完成后返回结果 (result.sequence)
        """
        # 获取客户端发送的目标长度 (Fibonacci.Goal.order)
        target_length = goal_handle.request.order
        
        self.get_logger().info(f'开始计算斐波那契数列，长度: {target_length}')
        
        # 创建反馈对象，用于发送进度信息 (Fibonacci.Feedback)
        feedback = Fibonacci.Feedback()
        
        # 初始化斐波那契数列
        fibonacci_sequence = []
        
        # 计算斐波那契数列
        for i in range(target_length):
            # 检查客户端是否请求取消任务
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('任务被客户端取消')
                return Fibonacci.Result()
            
            # 计算当前斐波那契数
            if i == 0:
                fibonacci_sequence.append(0)
            elif i == 1:
                fibonacci_sequence.append(1)
            else:
                # F(n) = F(n-1) + F(n-2)
                next_number = fibonacci_sequence[i-1] + fibonacci_sequence[i-2]
                fibonacci_sequence.append(next_number)
            
            # 设置当前进度反馈 (Fibonacci.Feedback.sequence)
            # 例如：i=3时，feedback.sequence = [0,1,1,2]
            feedback.sequence = fibonacci_sequence.copy()
            goal_handle.publish_feedback(feedback)
            
            # 打印当前进度
            current_number = fibonacci_sequence[-1]
            self.get_logger().info(f'计算第{i+1}个数: {current_number} (序列: {fibonacci_sequence})')
            
            # 模拟计算延迟，让客户端能看到进度变化
            time.sleep(0.5)
        
        # 任务成功完成
        goal_handle.succeed()
        
        # 创建并返回最终结果 (Fibonacci.Result)
        result = Fibonacci.Result()
        # 例如：target_length=6时，result.sequence = [0,1,1,2,3,5]
        result.sequence = fibonacci_sequence  # 完整的斐波那契数列
        
        self.get_logger().info(f'斐波那契数列计算完成: {fibonacci_sequence}')
        return result


def main(args=None):
    """
    主函数 - 启动Action服务端节点
    
    执行流程：
    1. 初始化ROS 2
    2. 创建Action服务端节点
    3. 运行节点，等待客户端请求
    4. 清理资源并退出
    """
    # 初始化ROS 2 Python客户端库
    rclpy.init(args=args)
    
    # 创建Action服务端节点实例
    server = SimpleActionServer()
    
    try:
        # 开始节点循环，等待Action请求
        rclpy.spin(server)
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        pass
    finally:
        # 清理资源
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 