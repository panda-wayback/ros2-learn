#!/usr/bin/env python3
"""
ROS 2 Action服务端示例 - 计数任务

功能说明：
- 接收客户端发送的目标数字（如：5）
- 从0开始计数到目标数字
- 每0.5秒发送一次进度反馈
- 完成后返回完整的计数序列

使用场景：
- 学习ROS 2 Action的基本用法
- 演示长时间任务的执行过程
- 展示反馈机制的使用

运行方法：
ros2 run learn_ros2 simple_action_server
"""

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer
from example_interfaces.action import Fibonacci
import time


class SimpleActionServer(Node):
    """
    简单Action服务端 - 计数任务执行器
    
    功能：
    - 接收计数目标（如：计数到5）
    - 执行计数任务（0,1,2,3,4,5）
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
        Action执行回调函数 - 核心计数逻辑
        
        参数：
        - goal_handle: 目标句柄，包含客户端发送的目标信息
        
        返回值：
        - result: 包含完整计数序列的结果对象
        
        执行流程：
        1. 获取目标数字
        2. 从0开始计数
        3. 每步发送进度反馈
        4. 完成后返回结果
        """
        # 获取客户端发送的目标数字
        target = goal_handle.request.order
        
        self.get_logger().info(f'开始计数到 {target}')
        
        # 创建反馈对象，用于发送进度信息
        feedback = Fibonacci.Feedback()
        
        # 从0开始计数到目标数字
        for i in range(target + 1):
            # 检查客户端是否请求取消任务
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('任务被客户端取消')
                return Fibonacci.Result()
            
            # 设置当前进度反馈（0到i的序列）
            feedback.sequence = list(range(i + 1))
            goal_handle.publish_feedback(feedback)
            
            # 打印当前进度
            self.get_logger().info(f'当前计数: {i}')
            
            # 模拟工作延迟，让客户端能看到进度变化
            time.sleep(0.5)
        
        # 任务成功完成
        goal_handle.succeed()
        
        # 创建并返回最终结果
        result = Fibonacci.Result()
        result.sequence = list(range(target + 1))  # 完整的计数序列
        
        self.get_logger().info('计数任务完成')
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