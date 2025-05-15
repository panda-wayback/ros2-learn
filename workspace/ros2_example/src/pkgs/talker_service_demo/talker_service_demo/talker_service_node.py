#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class TalkerServiceNode(Node):
    """
    一个同时具有多个话题发布和订阅功能的节点
    """
    def __init__(self):
        super().__init__('talker_service_node')
        
        # 创建第一个话题发布者
        self.publisher1 = self.create_publisher(String, 'chatter', 10)
        
        # 创建第二个话题发布者
        self.publisher2 = self.create_publisher(Int32, 'number', 10)
        
        # 创建话题订阅者
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        
        # 创建定时器，每1秒发布一次消息
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.counter = 0
        self.get_logger().info('节点已初始化')

    def timer_callback(self):
        """
        定时器回调函数，发布消息
        """
        # 发布字符串消息到 chatter 话题
        msg1 = String()
        msg1.data = 'Hello ROS 2!'
        self.publisher1.publish(msg1)
        self.get_logger().info(f'发布消息到 chatter: {msg1.data}')
        
        # 发布数字消息到 number 话题
        msg2 = Int32()
        msg2.data = self.counter
        self.publisher2.publish(msg2)
        self.get_logger().info(f'发布消息到 number: {msg2.data}')
        
        self.counter += 1

    def listener_callback(self, msg):
        """
        订阅回调函数，接收消息
        """
        self.get_logger().info(f'收到消息: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TalkerServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 