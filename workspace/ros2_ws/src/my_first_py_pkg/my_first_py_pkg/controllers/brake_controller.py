#!/usr/bin/env python3
from simple_pid import PID
from .base_controller import BaseController

class BrakeController(BaseController):
    """刹车控制器"""
    def __init__(self, speed_sensor=None):
        super().__init__('brake_controller')
        if speed_sensor is None:
            raise ValueError("必须提供速度传感器")
        self.speed_sensor = speed_sensor
        
        # 创建 PID 控制器
        self.pid = PID(
            Kp=1.0,   # 比例系数
            Ki=0.1,   # 积分系数
            Kd=0.3,   # 微分系数
            setpoint=0.0,  # 目标速度（刹车时总是希望速度为0）
            output_limits=(0.0, 1.0),  # 刹车输出限制（0-1）
            sample_time=0.1,  # 采样时间
            auto_mode=True  # 自动模式
        )
        
        # 设置积分项限制，防止积分饱和
        self.pid.Ki_limits = (-0.5, 0.5)

    def update(self, dt=0.1):
        """更新刹车控制
        
        Args:
            dt (float): 时间间隔（秒）
            
        Returns:
            float: 刹车力度（0-1）
        """
        if not self.is_enabled():
            return 0.0

        # 获取当前速度
        current_speed = self.speed_sensor.get_speed()
        current_acceleration = self.speed_sensor.get_acceleration()

        # 使用 PID 控制器计算刹车值
        # 目标速度始终为0（刹车的目的就是停下来）
        brake = self.pid(current_speed, dt=dt)
        
        # 如果加速度为负（已经在减速），适当减小刹车力度
        if current_acceleration < 0:
            brake *= 0.8
            
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