#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TemplateNode(Node):
    def __init__(self):
        super().__init__('template_node')
        self.get_logger().info('Template node has been started')
        
        # 在这里添加您的节点逻辑
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info('Template node is running')

def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 