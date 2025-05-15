#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SecondTalkerNode(Node):
    """
    第二个发布者节点，也发布到 chatter 话题
    """
    def __init__(self):
        super().__init__('second_talker_node')
        
        # 创建发布者，发布到同一个 chatter 话题
        self.publisher = self.create_publisher(String, 'chatter', 10)
        
        # 创建定时器，每2秒发布一次消息
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('第二个发布者节点已初始化')

    def timer_callback(self):
        """
        定时器回调函数，发布消息
        """
        msg = String()
        msg.data = '这是来自第二个发布者的消息!'
        self.publisher.publish(msg)
        self.get_logger().info(f'第二个发布者发布消息: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SecondTalkerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 