#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParamExample(Node):
    def __init__(self):
        super().__init__('param_example')
        # 声明参数，设置默认值
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('color', 'blue')
        self.declare_parameter('size', 1.0)
        
        # 创建定时器，每秒检查一次参数
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('参数示例节点已启动')

    def timer_callback(self):
        # 获取参数值
        speed = self.get_parameter('speed').value
        color = self.get_parameter('color').value
        size = self.get_parameter('size').value
        
        self.get_logger().info(f'当前参数值: speed={speed}, color={color}, size={size}')

def main(args=None):
    rclpy.init(args=args)
    node = ParamExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 