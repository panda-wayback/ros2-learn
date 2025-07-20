#!/usr/bin/env python3
"""
订阅者节点示例
订阅 'chatter' 话题并处理接收到的消息
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """
    订阅者节点类
    订阅指定话题并处理接收到的消息
    """
    
    def __init__(self):
        # 调用父类构造函数，设置节点名称
        super().__init__('subscriber_node')
        
        # 创建订阅者，订阅String类型消息从'chatter'话题
        # 参数说明：
        # - String: 消息类型（来自std_msgs包）
        # - 'chatter': 话题名称
        # - self.listener_callback: 消息接收回调函数
        # - 10: 队列大小（缓存最近10条消息）
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        
        # 消息接收计数器
        self.received_count = 0
        
        # 打印启动信息
        self.get_logger().info('订阅者节点已启动，正在监听 /chatter 话题...')
    
    def listener_callback(self, msg):
        """
        消息接收回调函数
        当接收到消息时自动调用此函数
        
        参数:
            msg: 接收到的消息对象（String类型）
        """
        # 增加接收计数器
        self.received_count += 1
        
        # 打印接收到的消息
        self.get_logger().info(
            f'收到第 {self.received_count} 条消息: "{msg.data}"'
        )
        
        # 可以在这里添加更多的消息处理逻辑
        # 例如：解析消息内容、触发其他操作等
        self.process_message(msg)
    
    def process_message(self, msg):
        """
        处理接收到的消息
        可以在这里添加自定义的消息处理逻辑
        
        参数:
            msg: 接收到的消息对象
        """
        # 示例：检查消息内容并做出相应处理
        if 'Hello ROS 2' in msg.data:
            self.get_logger().info('检测到问候消息！')
        
        # 示例：统计消息长度
        message_length = len(msg.data)
        self.get_logger().info(f'消息长度: {message_length} 个字符')


def main(args=None):
    """
    主函数
    初始化ROS 2，创建并运行订阅者节点
    """
    # 初始化ROS 2 Python客户端库
    rclpy.init(args=args)
    
    # 创建订阅者节点实例
    subscriber_node = SubscriberNode()
    
    try:
        # 开始节点循环，处理回调
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        pass
    finally:
        # 销毁节点
        subscriber_node.destroy_node()
        # 关闭ROS 2
        rclpy.shutdown()


if __name__ == '__main__':
    main() 