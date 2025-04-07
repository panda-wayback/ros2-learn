#!/usr/bin/env python3
import math
from .base_controller import BaseController

class MotionController(BaseController):
    """运动规划控制器，负责计算目标速度和方向"""
    def __init__(self):
        super().__init__('motion_controller')
        # 导航参数
        self.min_distance = 0.1      # 到达目标点的最小距离
        self.max_speed = 2.0         # 最大速度
        self.min_speed = 0.5         # 最小速度
        self.slow_down_distance = 2.0 # 开始减速的距离

    def compute_motion(self, target_x, target_y, current_x, current_y, current_angle):
        """计算运动控制指令
        
        Args:
            target_x (float): 目标点x坐标
            target_y (float): 目标点y坐标
            current_x (float): 当前x坐标
            current_y (float): 当前y坐标
            current_angle (float): 当前角度（弧度）
            
        Returns:
            tuple: (目标速度, 目标角度)
        """
        if not self.is_enabled():
            return 0.0, current_angle

        # 计算到目标点的距离和角度
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)

        # 如果已经到达目标点附近，停止移动
        if distance < self.min_distance:
            return 0.0, current_angle

        # 根据距离计算目标速度
        target_speed = self.compute_target_speed(distance)
        
        return target_speed, target_angle

    def compute_target_speed(self, distance):
        """根据距离计算目标速度
        
        Args:
            distance (float): 到目标点的距离
            
        Returns:
            float: 目标速度
        """
        if distance < self.min_distance:
            return 0.0
        
        # 在接近目标点时逐渐减速
        if distance < self.slow_down_distance:
            speed_factor = distance / self.slow_down_distance
            target_speed = self.min_speed + (self.max_speed - self.min_speed) * speed_factor
        else:
            target_speed = self.max_speed
            
        return target_speed

    def reset(self):
        """重置控制器状态"""
        pass  # 运动规划控制器不需要维护状态 