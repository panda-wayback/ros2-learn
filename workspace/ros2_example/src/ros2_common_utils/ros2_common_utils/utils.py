#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from typing import Any, Dict, List, Optional, Tuple, Union

class ROS2Utils:
    """ROS2通用工具函数类"""
    
    @staticmethod
    def create_qos_profile(
        reliability: str = 'reliable',
        history: str = 'keep_last',
        depth: int = 10
    ) -> QoSProfile:
        """
        创建QoS配置文件
        
        Args:
            reliability: 可靠性策略 ('reliable' 或 'best_effort')
            history: 历史策略 ('keep_last' 或 'keep_all')
            depth: 历史深度
            
        Returns:
            QoSProfile: QoS配置文件
        """
        reliability_policy = (
            ReliabilityPolicy.RELIABLE if reliability == 'reliable'
            else ReliabilityPolicy.BEST_EFFORT
        )
        history_policy = (
            HistoryPolicy.KEEP_LAST if history == 'keep_last'
            else HistoryPolicy.KEEP_ALL
        )
        
        return QoSProfile(
            reliability=reliability_policy,
            history=history_policy,
            depth=depth
        )
    
    @staticmethod
    def get_node_name(node: Node) -> str:
        """
        获取节点名称
        
        Args:
            node: ROS2节点实例
            
        Returns:
            str: 节点名称
        """
        return node.get_name()
    
    @staticmethod
    def get_topic_name(node: Node, topic: str) -> str:
        """
        获取完整的主题名称
        
        Args:
            node: ROS2节点实例
            topic: 主题名称
            
        Returns:
            str: 完整的主题名称
        """
        return f"{node.get_name()}/{topic}"
    
    @staticmethod
    def create_timer(
        node: Node,
        period: float,
        callback: callable
    ) -> rclpy.timer.Timer:
        """
        创建定时器
        
        Args:
            node: ROS2节点实例
            period: 定时器周期（秒）
            callback: 回调函数
            
        Returns:
            Timer: 定时器实例
        """
        return node.create_timer(period, callback)
    
    @staticmethod
    def log_info(node: Node, message: str) -> None:
        """
        记录信息日志
        
        Args:
            node: ROS2节点实例
            message: 日志消息
        """
        node.get_logger().info(message)
    
    @staticmethod
    def log_warn(node: Node, message: str) -> None:
        """
        记录警告日志
        
        Args:
            node: ROS2节点实例
            message: 日志消息
        """
        node.get_logger().warn(message)
    
    @staticmethod
    def log_error(node: Node, message: str) -> None:
        """
        记录错误日志
        
        Args:
            node: ROS2节点实例
            message: 日志消息
        """
        node.get_logger().error(message) 