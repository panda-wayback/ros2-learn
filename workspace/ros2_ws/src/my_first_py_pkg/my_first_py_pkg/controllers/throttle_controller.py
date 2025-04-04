#!/usr/bin/env python3
from .base_controller import BaseController

class ThrottleController(BaseController):
    """油门控制器"""
    def __init__(self, max_speed=0.5):
        super().__init__('throttle_controller')
        self.max_speed = max_speed
        self.current_throttle = 0.0
        self.target_speed = 0.0
        
        # PID控制参数
        self.kp = 0.5  # 比例系数
        self.ki = 0.1  # 积分系数
        self.kd = 0.2  # 微分系数
        
        # PID状态
        self.error_sum = 0.0
        self.last_error = 0.0

    def set_target_speed(self, speed):
        """设置目标速度"""
        self.target_speed = min(max(0.0, speed), self.max_speed)

    def update(self, current_speed, dt=0.1):
        """使用PID控制更新油门值"""
        if not self.is_enabled():
            return 0.0

        # 计算速度误差
        error = self.target_speed - current_speed
        
        # 更新积分项
        self.error_sum += error * dt
        # 防止积分饱和
        self.error_sum = min(max(-1.0, self.error_sum), 1.0)
        
        # 计算微分项
        error_diff = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        
        # PID控制计算油门值
        throttle = (
            self.kp * error +          # 比例项
            self.ki * self.error_sum + # 积分项
            self.kd * error_diff       # 微分项
        )
        
        # 限制油门范围
        self.current_throttle = min(max(0.0, throttle), 1.0)
        return self.current_throttle

    def reset(self):
        """重置油门控制器状态"""
        self.current_throttle = 0.0
        self.target_speed = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0 