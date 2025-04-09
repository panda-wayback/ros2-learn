#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(SetBool, 'toggle_service', self.toggle_callback)
        self.get_logger().info('Service node is ready!')

    def toggle_callback(self, request, response):
        self.get_logger().info(f'Received request: {request.data}')
        response.success = True
        response.message = 'Service call successful!'
        return response

def main(args=None):
    rclpy.init(args=args)
    service_node = ServiceNode()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 