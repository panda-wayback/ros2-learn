#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2服务节点示例
这个节点实现了一个简单的布尔值切换服务
客户端可以发送布尔值请求，服务器会处理并返回响应
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # 导入标准服务消息类型
from common_interfaces.msg import Status, Command

class ServiceNode(Node):
    """
    服务节点类
    继承自ROS2的Node类，实现服务端功能
    """
    def __init__(self):
        """
        初始化服务节点
        - 创建节点名称：'service_node'
        - 创建服务：'toggle_service'
        - 设置回调函数：toggle_callback
        """
        super().__init__('service_node')
        # 创建服务，参数说明：
        # 1. SetBool: 服务消息类型（包含请求和响应）
        # 2. 'toggle_service': 服务名称
        # 3. self.toggle_callback: 处理请求的回调函数
        self.srv = self.create_service(SetBool, 'toggle_service', self.toggle_callback)
        self.get_logger().info('Service node is ready!')

    def toggle_callback(self, request, response):
        """
        服务回调函数
        处理客户端请求并返回响应

        参数:
            request: 客户端发送的请求对象
                - request.data: 布尔值，表示开关状态
            response: 要返回给客户端的响应对象
                - response.success: 布尔值，表示操作是否成功
                - response.message: 字符串，描述操作结果

        返回:
            response: 处理后的响应对象
        """
        # 记录接收到的请求数据
        self.get_logger().info(f'Received request: {request.data}')
        
        # 设置响应数据
        response.success = True  # 操作成功
        response.message = 'Service call successful!'  # 成功消息
        
        return response

def main(args=None):
    """
    主函数
    初始化ROS2，创建并运行服务节点
    """
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建服务节点实例
    service_node = ServiceNode()
    
    # 运行节点，等待请求
    # spin()会阻塞当前线程，直到节点被关闭
    rclpy.spin(service_node)
    
    # 清理资源
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 