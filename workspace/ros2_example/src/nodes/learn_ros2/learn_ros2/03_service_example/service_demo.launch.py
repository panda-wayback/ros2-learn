#!/usr/bin/env python3
"""
ROS 2 服务演示启动文件
同时启动服务端和客户端进行演示
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成启动描述
    """
    return LaunchDescription([
        # 启动服务端节点
        Node(
            package='learn_ros2',
            executable='service_server',
            name='add_two_ints_server',
            output='screen',
            emulate_tty=True,
        ),
        
        # 启动自动测试客户端节点（延迟2秒启动）
        Node(
            package='learn_ros2',
            executable='service_client',
            name='add_two_ints_client',
            output='screen',
            emulate_tty=True,
            # 延迟启动，确保服务端先启动
            parameters=[{'delay_seconds': 2}],
        ),
    ]) 