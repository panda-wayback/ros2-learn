#!/usr/bin/env python3
"""
发布者节点示例
发布字符串消息到 'chatter' 话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class PublisherNode(Node):
    """
    发布者节点类
    定期发布消息到指定话题
    """
    
    def __init__(self):
        # 调用父类构造函数，设置节点名称
        super().__init__('publisher_node')
        
        # 创建发布者，发布String类型消息到'chatter'话题
        # 参数说明：
        # - String: 消息类型（来自std_msgs包）
        # - 'chatter': 话题名称
        # - 10: 队列大小（缓存最近10条消息）
        self.publisher = self.create_publisher(String, 'chatter', 10)
        
        # 创建定时器，每1秒调用一次timer_callback函数
        # 参数说明：
        # - 1.0: 时间间隔（秒）
        # - self.timer_callback: 回调函数
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 消息计数器
        self.count = 0
        
        # 打印启动信息
        self.get_logger().info('发布者节点已启动，正在发布消息到 /chatter 话题...')
    
    def timer_callback(self):
        """
        定时器回调函数
        创建并发布消息
        """
        # 创建String消息对象
        msg = String()
        
        # 设置消息内容
        msg.data = f'Hello ROS 2! 这是第 {self.count} 条消息'
        
        # 发布消息
        self.publisher.publish(msg)
        
        # 打印发布信息
        self.get_logger().info(f'已发布: "{msg.data}"')
        
        # 计数器递增
        self.count += 1


def main(args=None):
    """
    主函数
    初始化ROS 2，创建并运行发布者节点
    """
    # 初始化ROS 2 Python客户端库
    rclpy.init(args=args)
    
    # 创建发布者节点实例
    publisher_node = PublisherNode()
    
    try:
        # 开始节点循环，处理回调
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        pass
    finally:
        # 销毁节点
        publisher_node.destroy_node()
        # 关闭ROS 2
        rclpy.shutdown()


if __name__ == '__main__':
    main() 