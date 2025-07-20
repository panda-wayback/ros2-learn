#!/usr/bin/env python3
"""
CLI工具演示 - 服务端节点

本节点旨在演示如何使用ROS2的命令行工具(CLI)进行调试。
它会创建一个简单的加法服务，并在每次被调用时，通过一个Topic发布计算历史。
这样我们就可以用:
- `ros2 service call` 来调用服务
- `ros2 topic echo` 来查看发布的话题
"""
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String

class CLIServiceServer(Node):
    def __init__(self):
        super().__init__('cli_service_server')

        # 1. 创建一个Service Server
        #    服务名称: /add_two_ints
        #    服务类型: AddTwoInts (接收两个整数 a, b, 返回一个整数 sum)
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )
        self.get_logger().info("创建服务 '/add_two_ints'")

        # 2. 创建一个Publisher
        #    话题名称: /calc_history
        #    作用：每次服务被调用时，都把计算过程发布出去
        self.history_publisher = self.create_publisher(
            String,
            'calc_history',
            10 # QoS depth
        )
        self.get_logger().info("创建话题 '/calc_history'")
        self.get_logger().info("\n--- 服务端已准备就绪 ---")

    def add_two_ints_callback(self, request, response):
        """
        服务的回调函数。当收到请求时，此函数被执行。
        """
        # 从请求中获取输入的两个整数
        a = request.a
        b = request.b
        
        # 计算和
        response.sum = a + b
        
        self.get_logger().info(f"收到请求: a={a}, b={b}。 正在返回结果: sum={response.sum}")

        # 创建并发布计算历史消息
        history_msg = String()
        history_msg.data = f"计算历史: {a} + {b} = {response.sum}"
        self.history_publisher.publish(history_msg)
        self.get_logger().info(f"已将 '{history_msg.data}' 发布到 /calc_history")

        # 返回响应
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CLIServiceServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("服务端节点被关闭。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 