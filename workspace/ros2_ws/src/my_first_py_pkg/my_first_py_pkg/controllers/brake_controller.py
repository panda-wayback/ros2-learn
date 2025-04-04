#!/usr/bin/env python3
from .base_controller import BaseController

class BrakeController(BaseController):
    """刹车控制器"""
    def __init__(self):
        super().__init__('brake_controller')
        self.current_brake = 0.0
        self.target_brake = 0.0
        self.brake_rate = 0.2  # 刹车速率（单位/秒）

    def set_brake_force(self, force):
        """设置刹车力度"""
        self.target_brake = min(max(0.0, force), 1.0)

    def update(self, dt=0.1):
        """更新刹车状态"""
        if not self.is_enabled():
            return 0.0

        # 计算刹车力度差
        brake_diff = self.target_brake - self.current_brake
        
        # 计算这一帧的刹车力度变化
        brake_change = min(abs(brake_diff), self.brake_rate * dt)
        if brake_diff < 0:
            brake_change = -brake_change
            
        # 更新当前刹车力度
        self.current_brake += brake_change
        return self.current_brake

    def reset(self):
        """重置刹车控制器状态"""
        self.current_brake = 0.0
        self.target_brake = 0.0 