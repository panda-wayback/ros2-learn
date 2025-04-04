#!/usr/bin/env python3
import math
import time
from .base_sensor import BaseSensor

class GyroSensor(BaseSensor):
    """陀螺仪传感器，用于测量角度和角速度"""
    def __init__(self):
        super().__init__('gyro_sensor')
        self.current_angle = 0.0  # 当前角度（弧度）
        self.angular_velocity = 0.0  # 当前角速度（弧度/秒）
        self.last_angle = 0.0  # 上一次的角度
        self.last_update_time = time.time()  # 上一次更新的时间
        self.min_dt = 0.001  # 最小时间间隔（秒）

    def update(self, angle, angular_velocity=None):
        """更新传感器状态
        
        Args:
            angle (float): 当前角度（弧度）
            angular_velocity (float, optional): 当前角速度（弧度/秒）。
                如果不提供，将通过角度差分计算。
        """
        current_time = time.time()
        dt = max(current_time - self.last_update_time, self.min_dt)
        
        # 标准化角度到 [-pi, pi] 范围
        self.current_angle = self.normalize_angle(angle)
        
        if angular_velocity is not None:
            # 如果提供了角速度，直接使用
            self.angular_velocity = angular_velocity
        else:
            # 否则通过角度差分计算角速度
            angle_diff = self.normalize_angle_diff(self.current_angle - self.last_angle)
            self.angular_velocity = angle_diff / dt
        
        self.last_angle = self.current_angle
        self.last_update_time = current_time

    def get_angle(self):
        """获取当前角度（弧度）"""
        return self.current_angle

    def get_angular_velocity(self):
        """获取当前角速度（弧度/秒）"""
        return self.angular_velocity

    def reset(self):
        """重置传感器状态"""
        self.current_angle = 0.0
        self.angular_velocity = 0.0
        self.last_angle = 0.0
        self.last_update_time = time.time()

    @staticmethod
    def normalize_angle(angle):
        """将角度标准化到 [-pi, pi] 范围内
        
        Args:
            angle (float): 输入角度（弧度）
            
        Returns:
            float: 标准化后的角度（弧度）
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def normalize_angle_diff(angle_diff):
        """标准化角度差，使其在 [-pi, pi] 范围内
        
        Args:
            angle_diff (float): 角度差（弧度）
            
        Returns:
            float: 标准化后的角度差（弧度）
        """
        while angle_diff > math.pi:
            angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2.0 * math.pi
        return angle_diff 