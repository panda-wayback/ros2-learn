#!/usr/bin/env python3
"""
ROS 2 服务客户端节点示例
演示如何创建一个客户端，调用加法计算服务

服务客户端特点：
1. 发送请求：向服务端发送数据
2. 等待响应：同步等待服务端处理完成
3. 获取结果：从服务端接收计算结果
4. 阻塞调用：调用期间会阻塞，直到收到响应
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServiceClient(Node):
    """
    加法计算服务客户端节点
    
    功能：
    - 调用加法计算服务
    - 发送两个整数，接收计算结果
    - 演示ROS 2服务客户端的基本用法
    """
    
    def __init__(self):
        super().__init__('add_two_ints_client')
        
        # 创建服务客户端
        # 参数说明：
        # - AddTwoInts: 服务类型
        # - 'add_two_ints': 服务名称（必须与服务端一致）
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 等待服务端可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务端启动...')
        
        self.get_logger().info('服务端已连接，可以发送请求')
    
    def send_request(self, a, b):
        """
        发送服务请求
        
        参数：
        - a: 第一个整数
        - b: 第二个整数
        
        返回值：
        - response: 服务响应对象，包含计算结果
        """
        # 创建请求对象
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # 发送请求并等待响应
        # 这是一个同步调用，会阻塞直到收到响应
        future = self.client.call_async(request)
        
        # 等待响应完成
        rclpy.spin_until_future_complete(self, future)
        
        # 获取响应结果
        response = future.result()
        
        return response


def main(args=None):
    """
    主函数
    """
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建客户端节点
    add_two_ints_client = AddTwoIntsServiceClient()
    
    try:
        # 发送几个测试请求
        test_cases = [
            (1, 2),
            (10, 20),
            (100, 200),
            (-5, 8)
        ]
        
        for a, b in test_cases:
            # 发送请求
            response = add_two_ints_client.send_request(a, b)
            
            # 打印结果
            add_two_ints_client.get_logger().info(
                f'请求: {a} + {b} = {response.sum}'
            )
        
        # 等待一段时间，让用户看到结果
        add_two_ints_client.get_logger().info('所有测试完成')
        
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        pass
    finally:
        # 清理资源
        add_two_ints_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 