#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_common_utils.utils import ROS2Utils

class UtilsExampleNode(Node):
    """使用工具函数的示例节点"""
    
    def __init__(self):
        super().__init__('utils_example_node')
        
        # 使用工具函数记录日志
        ROS2Utils.log_info(self, "节点初始化开始")
        
        # 使用工具函数创建QoS配置
        qos_profile = ROS2Utils.create_qos_profile(
            reliability='reliable',
            history='keep_last',
            depth=10
        )
        
        # 创建发布者
        self.publisher = self.create_publisher(
            String,
            'example_topic',
            qos_profile
        )
        
        # 使用工具函数创建定时器
        self.timer = ROS2Utils.create_timer(
            self,
            period=1.0,  # 1秒
            callback=self.timer_callback
        )
        
        # 使用工具函数获取节点名称
        node_name = ROS2Utils.get_node_name(self)
        ROS2Utils.log_info(self, f"节点名称: {node_name}")
        
        # 使用工具函数获取完整主题名称
        topic_name = ROS2Utils.get_topic_name(self, 'example_topic')
        ROS2Utils.log_info(self, f"主题名称: {topic_name}")
        
        ROS2Utils.log_info(self, "节点初始化完成")
    
    def timer_callback(self):
        """定时器回调函数"""
        try:
            # 创建消息
            msg = String()
            msg.data = f"Hello from {ROS2Utils.get_node_name(self)}"
            
            # 发布消息
            self.publisher.publish(msg)
            ROS2Utils.log_info(self, f"已发布消息: {msg.data}")
            
        except Exception as e:
            # 使用工具函数记录错误
            ROS2Utils.log_error(self, f"发布消息时发生错误: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = UtilsExampleNode()
    
    try:
        # 运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        ROS2Utils.log_info(node, "节点被用户中断")
    except Exception as e:
        ROS2Utils.log_error(node, f"节点运行时发生错误: {str(e)}")
    finally:
        # 清理
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 