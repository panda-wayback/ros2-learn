#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from example_pkg.msg import RobotStatus


class RobotStatusNode(Node):
    """
    使用自定义消息的机器人状态节点
    """
    # 定义话题名
    # 定义话题名
    ROBOT_STATUS_TOPIC = 'robot_status'
    
    def __init__(self):
        super().__init__('robot_status_node')
        
        # 创建发布者，使用自定义消息类型
        self.publisher = self.create_publisher(RobotStatus,self.ROBOT_STATUS_TOPIC, 10)
        
        # 创建定时器，每1秒发布一次消息
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.counter = 0
        self.get_logger().info('机器人状态节点已初始化')

    def timer_callback(self):
        """
        定时器回调函数，发布机器人状态消息
        """
        msg = RobotStatus()
        msg.robot_name = 'robot1'
        msg.battery_level = 100 - (self.counter % 100)
        msg.x = float(self.counter)
        msg.y = float(self.counter * 2)
        msg.is_connected = True
        
        self.publisher.publish(msg)
        self.get_logger().info(f'发布机器人状态: {msg.robot_name}, 电量: {msg.battery_level}%')
        
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 