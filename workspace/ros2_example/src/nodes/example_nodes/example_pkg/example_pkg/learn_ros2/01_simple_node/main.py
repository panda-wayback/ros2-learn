#!/usr/bin/env python3
"""
01_simple_node - 最简单的ROS2节点示例

这个节点演示了ROS2节点的基本结构：
1. 导入必要的ROS2库
2. 创建节点类
3. 初始化节点
4. 运行节点

学习目标：
- 理解ROS2节点的基本概念
- 掌握节点的创建和运行流程
- 了解节点的生命周期
"""

import rclpy
from rclpy.node import Node


class SimpleNode(Node):
    """
    最简单的ROS2节点类
    
    这个类继承自Node基类，实现了最基本的节点功能：
    - 节点初始化
    - 基本的日志输出
    - 定时器回调
    """
    
    def __init__(self):
        """
        节点初始化函数
        
        在这里我们：
        1. 调用父类构造函数，设置节点名称
        2. 创建定时器，定期执行回调函数
        3. 输出初始化日志
        """
        # 调用父类构造函数，设置节点名称为 'simple_node'
        # 节点名称在ROS2系统中必须是唯一的
        super().__init__('simple_node')
        
        # 输出初始化日志
        self.get_logger().info('01_simple_node 已启动！')
        self.get_logger().info('这是一个最简单的ROS2节点示例')
        
        # 创建定时器，每1秒执行一次timer_callback函数
        # 定时器是ROS2中常用的机制，用于周期性执行任务
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 输出定时器创建成功的日志
        self.get_logger().info('定时器已创建，每秒执行一次回调')
    
    def timer_callback(self):
        """
        定时器回调函数
        
        这个函数会被定时器定期调用，用于：
        - 输出心跳信息
        - 演示节点的持续运行
        - 显示当前时间戳
        """
        # 获取当前时间戳
        current_time = self.get_clock().now()
        
        # 输出心跳信息
        self.get_logger().info(f'节点正在运行... 时间戳: {current_time}')


def main(args=None):
    """
    主函数 - 节点的入口点
    
    这个函数负责：
    1. 初始化ROS2客户端库
    2. 创建节点实例
    3. 运行节点
    4. 处理异常和清理资源
    """
    # 初始化ROS2客户端库
    # 这是使用ROS2功能的必要步骤
    rclpy.init(args=args)
    
    try:
        # 创建SimpleNode实例
        simple_node = SimpleNode()
        
        # 输出启动信息
        simple_node.get_logger().info('开始运行节点...')
        
        # 运行节点，开始处理回调
        # spin()函数会让节点持续运行，直到被中断
        rclpy.spin(simple_node)
        
    except KeyboardInterrupt:
        # 处理键盘中断（Ctrl+C）
        print('\n收到中断信号，正在关闭节点...')
        
    except Exception as e:
        # 处理其他异常
        print(f'节点运行出错: {e}')
        
    finally:
        # 清理资源
        # 销毁节点实例
        if 'simple_node' in locals():
            simple_node.destroy_node()
        
        # 关闭ROS2客户端库
        rclpy.shutdown()
        
        print('节点已关闭，程序结束')


if __name__ == '__main__':
    """
    程序入口点
    
    当直接运行这个Python文件时，会执行main()函数
    这是Python的标准做法
    """
    main() 