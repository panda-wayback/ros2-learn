#!/usr/bin/env python3

# 导入必要的ROS2 Python客户端库
import rclpy
from rclpy.node import Node
# 导入标准服务消息类型 SetBool
from std_srvs.srv import SetBool
from example_pkg.constants import SERVICE_TOGGLE, NODE_SERVICE

class ServiceNode(Node):
    """
    服务节点类，继承自ROS2的Node类
    这个节点提供了一个布尔值切换服务
    """
    def __init__(self):
        # 调用父类构造函数，设置节点名称为'service_node'
        super().__init__(NODE_SERVICE)
        # 创建服务，使用SetBool服务类型，服务名称为'toggle_service'
        # toggle_callback 是处理服务请求的回调函数
        self.srv = self.create_service(SetBool, SERVICE_TOGGLE, self.toggle_callback)
        # 输出日志信息，表明服务节点已经准备就绪
        self.get_logger().info('Service node is ready!')

    def toggle_callback(self, request, response):
        """
        服务回调函数，处理服务请求
        Args:
            request: 服务请求对象，包含data字段（布尔值）
            response: 服务响应对象，包含success和message字段
        Returns:
            response: 处理后的响应对象
        """
        # 记录接收到的请求数据
        self.get_logger().info(f'Received request: {request.data}')
        # 设置响应成功标志
        response.success = True
        # 设置响应消息
        response.message = 'Service call successful!'
        return response

def main(args=None):
    """
    主函数，用于初始化和运行服务节点
    Args:
        args: 命令行参数，默认为None
    """
    # 初始化ROS2客户端库
    rclpy.init(args=args)
    # 创建服务节点实例
    service_node = ServiceNode()
    # 保持节点运行，等待服务请求
    rclpy.spin(service_node)
    # 清理节点资源
    service_node.destroy_node()
    # 关闭ROS2客户端库
    rclpy.shutdown()

if __name__ == '__main__':
    main() 