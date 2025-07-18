#!/usr/bin/env python3
"""
话题和消息演示启动文件
同时启动发布者和订阅者节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成启动描述
    返回包含所有需要启动节点的LaunchDescription对象
    """
    
    # 创建发布者节点
    publisher_node = Node(
        package='learn_ros2',  # 包名
        executable='publisher_node',  # 可执行文件名
        name='publisher_node',  # 节点名称
        output='screen',  # 输出到屏幕
        parameters=[],  # 参数列表（这里为空）
        arguments=[],  # 命令行参数（这里为空）
        respawn=True,  # 如果节点崩溃，自动重启
        respawn_delay=1.0  # 重启延迟时间（秒）
    )
    
    # 创建订阅者节点
    subscriber_node = Node(
        package='learn_ros2',  # 包名
        executable='subscriber_node',  # 可执行文件名
        name='subscriber_node',  # 节点名称
        output='screen',  # 输出到屏幕
        parameters=[],  # 参数列表（这里为空）
        arguments=[],  # 命令行参数（这里为空）
        respawn=True,  # 如果节点崩溃，自动重启
        respawn_delay=1.0  # 重启延迟时间（秒）
    )
    
    # 创建启动描述对象
    launch_description = LaunchDescription()
    
    # 添加节点到启动描述
    launch_description.add_action(publisher_node)
    launch_description.add_action(subscriber_node)
    
    return launch_description 