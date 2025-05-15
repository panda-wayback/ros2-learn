#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_pkg.nodes.publisher_node.main import PublisherNode


class SubscriberNode(Node):
    """
    订阅者节点类
    继承自ROS2的Node基类，用于创建一个订阅者节点
    """
    NODE_NAME = 'subscriber_node'

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.subscription = self.create_subscription(
            String,
            PublisherNode.TOPIC_NAME,  # 使用发布者节点定义的话题名称
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 