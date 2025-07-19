#!/usr/bin/env python3
"""
ROS 2 服务端节点示例
演示如何创建一个服务端，提供加法计算功能

服务（Service）特点：
1. 请求-响应模式：客户端发送请求，服务端返回响应
2. 同步通信：客户端会等待服务端处理完成
3. 适合一次性任务：如计算、查询、状态检查等
4. 有返回值：与话题不同，服务有明确的返回值
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServiceServer(Node):
    """
    加法计算服务端节点
    
    功能：
    - 提供加法计算服务
    - 接收两个整数，返回它们的和
    - 演示ROS 2服务的基本用法
    """
    
    def __init__(self):
        super().__init__('add_two_ints_server')
        
        # 创建服务端
        # 参数说明：
        # - 'add_two_ints': 服务名称
        # - AddTwoInts: 服务类型（包含请求和响应消息）
        # - AddTwoInts 是interface文件中定义的 service 类型 
        # - 在interface文件中，service 类型定义了请求和响应的消息类型
        # <l/setup.bash && ros2 interface show example_interfaces/srv/AddTwoInts
        #     int64 a
        #     int64 b
        #     ---
        #     int64 sum
        # - self.add_two_ints_callback: 回调函数，处理服务请求
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )
        
        self.get_logger().info('加法计算服务端已启动，等待请求...')
    
    def add_two_ints_callback(self, request, response):
        """
        服务回调函数
        
        参数：
        - request: 服务请求对象，包含客户端发送的数据
        - response: 服务响应对象，用于返回结果给客户端
        
        返回值：
        - response: 包含计算结果的响应对象
        """
        # 从请求中获取两个整数
        a = request.a
        b = request.b
        
        # 执行加法计算
        result = a + b
        
        # 将结果设置到响应中
        response.sum = result
        
        # 打印日志信息
        self.get_logger().info(f'收到请求: {a} + {b} = {result}')
        
        # 返回响应
        return response


def main(args=None):
    """
    主函数
    """
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建服务端节点
    add_two_ints_server = AddTwoIntsServiceServer()
    
    try:
        # 运行节点，等待服务请求
        rclpy.spin(add_two_ints_server)
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        pass
    finally:
        # 清理资源
        add_two_ints_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 