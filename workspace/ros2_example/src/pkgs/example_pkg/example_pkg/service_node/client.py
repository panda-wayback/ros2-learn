#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2服务客户端示例
这个节点演示如何调用服务节点
客户端可以发送布尔值请求并接收响应
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # 导入标准服务消息类型
import time  # 添加time模块

class ServiceClient(Node):
    """
    服务客户端类
    继承自ROS2的Node类，实现客户端功能
    """
    def __init__(self):
        """
        初始化客户端节点
        - 创建节点名称：'service_client'
        - 创建服务客户端：'toggle_service'
        """
        super().__init__('service_client')
        # 创建服务客户端
        self.client = self.create_client(SetBool, 'toggle_service')
        self.get_logger().info('Service client is ready!')

    def send_request(self, data):
        """
        发送服务请求并等待响应

        参数:
            data: 布尔值，要发送的请求数据

        返回:
            response: 服务端返回的响应对象
        """
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务可用...')

        # 创建请求对象
        request = SetBool.Request()
        request.data = data

        # 发送请求并等待响应
        self.get_logger().info(f'发送请求: {data}')
        future = self.client.call_async(request)
        
        # 等待响应
        try:
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(f'收到响应: 成功={response.success}, 消息={response.message}')
            return response
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {str(e)}')
            return None

def main(args=None):
    """
    主函数
    初始化ROS2，创建客户端并发送请求
    """
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建客户端节点
    client = ServiceClient()
    
    try:
        # 发送true请求
        response = client.send_request(True)
        if response and response.success:
            print("第一个请求成功！")
        
        # 等待2秒
        time.sleep(2.0)  # 使用time.sleep替代rclpy.sleep
        
        # 发送false请求
        response = client.send_request(False)
        if response and response.success:
            print("第二个请求成功！")
            
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        # 清理资源
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 