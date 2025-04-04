#!/usr/bin/env python3
from simple_pid import PID
from .base_controller import BaseController

class BrakeController(BaseController):
    """刹车控制器"""
    def __init__(self):
        super().__init__('brake_controller')
        self.target_speed = 0.0
        
        # 创建 PID 控制器
        self.pid = PID(
            Kp=1.0,   # 比例系数
            Ki=0.1,   # 积分系数
            Kd=0.3,   # 微分系数
            setpoint=0.0,  # 初始目标速度
            output_limits=(0.0, 1.0),  # 刹车输出限制（0-1）
            sample_time=0.1,  # 采样时间
            auto_mode=True  # 自动模式
        )
        
        # 设置积分项限制
        self.pid.Ki_limits = (-0.5, 0.5)

    def update(self, current_speed, dt=0.1):
        """使用PID控制更新刹车值"""
        if not self.is_enabled():
            return 0.0

        # 使用 PID 控制器计算刹车值
        # 注意：我们希望速度降到0，所以目标速度始终为0
        brake = self.pid(current_speed, dt=dt)
        return brake

    def reset(self):
        """重置刹车控制器状态"""
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