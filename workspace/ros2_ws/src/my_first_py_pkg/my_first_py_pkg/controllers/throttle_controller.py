#!/usr/bin/env python3
from simple_pid import PID
from .base_controller import BaseController

class ThrottleController(BaseController):
    """油门控制器"""
    def __init__(self, max_speed=0.5):
        super().__init__('throttle_controller')
        self.max_speed = max_speed
        self.target_speed = 0.0
        
        # 创建 PID 控制器
        self.pid = PID(
            Kp=0.8,   # 比例系数
            Ki=0.1,   # 积分系数
            Kd=0.2,   # 微分系数
            setpoint=0.0,  # 初始目标速度
            output_limits=(0.0, 1.0),  # 油门输出限制（0-1）
            sample_time=0.1,  # 采样时间
            auto_mode=True  # 自动模式
        )
        
        # 设置积分项限制，防止积分饱和
        self.pid.Ki_limits = (-0.5, 0.5)

    def set_target_speed(self, speed):
        """设置目标速度"""
        self.target_speed = min(max(0.0, speed), self.max_speed)
        self.pid.setpoint = self.target_speed

    def update(self, current_speed, dt=0.1):
        """使用PID控制更新油门值"""
        if not self.is_enabled():
            return 0.0

        # 使用 PID 控制器计算油门值
        throttle = self.pid(current_speed, dt=dt)
        return throttle

    def reset(self):
        """重置油门控制器状态"""
        self.target_speed = 0.0
        self.pid.reset()
        self.pid.setpoint = 0.0

    def tune_pid(self, kp=None, ki=None, kd=None):
        """调整PID参数"""
        if kp is not None:
            self.pid.Kp = kp
        if ki is not None:
            self.pid.Ki = ki
        if kd is not None:
            self.pid.Kd = kd 