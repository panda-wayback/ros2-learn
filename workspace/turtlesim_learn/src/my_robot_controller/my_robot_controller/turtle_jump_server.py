#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute

class TurtleJumpServer(Node):
    def __init__(self):
        super().__init__('turtle_jump_server')
        self.srv = self.create_service(
            TeleportAbsolute, 
            'turtle_jump', 
            self.jump_callback)
        self.get_logger().info('乌龟跳跃服务已启动')
        
        # 创建客户端用于控制乌龟
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待乌龟服务...')

    def jump_callback(self, request, response):
        self.get_logger().info(f'收到跳跃请求: x={request.x}, y={request.y}, theta={request.theta}')
        
        # 调用乌龟模拟器的服务
        req = TeleportAbsolute.Request()
        req.x = request.x
        req.y = request.y
        req.theta = request.theta
        
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('乌龟跳跃成功')
        else:
            self.get_logger().error('乌龟跳跃失败')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleJumpServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 