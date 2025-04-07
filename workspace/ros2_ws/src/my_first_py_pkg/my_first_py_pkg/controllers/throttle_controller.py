#!/usr/bin/env python3
from simple_pid import PID
from .base_controller import BaseController

class ThrottleController(BaseController):
    """油门控制器
    
    功能：
    1. 接收0~1的油门输入值
    2. 使用PID平滑处理，防止油门突变
    3. 输出平滑后的0~1油门值
    """
    def __init__(self, speed_sensor=None):
        super().__init__('throttle_controller')
        self.speed_sensor = speed_sensor  # 可选的速度传感器，用于调试
        
        # PID控制器配置
        self.pid = PID(
            Kp=1.0,   # 比例系数（决定响应速度）
            Ki=0.1,   # 积分系数（消除静态误差）
            Kd=0.05,  # 微分系数（抑制突变）
            setpoint=0.0,
            output_limits=(0.0, 1.0),
            sample_time=0.05,
            auto_mode=True
        )
        self.pid.Ki_limits = (-0.3, 0.3)  # 防止积分饱和

    def compute(self, target_throttle):
        """计算平滑后的油门输出值
        
        Args:
            target_throttle (float): 目标油门值，范围0~1
            
        Returns:
            float: 平滑处理后的油门值，范围0~1
        """
        if not self.is_enabled():
            return 0.0
            
        # 限制输入范围
        target_throttle = min(max(0.0, target_throttle), 1.0)
        self.pid.setpoint = target_throttle
        
        # 获取当前油门值
        current_throttle = self.pid._last_output if self.pid._last_output is not None else 0.0
        
        # 计算平滑后的油门值
        smooth_throttle = self.pid(current_throttle)
        
        # 记录调试信息（如果有速度传感器）
        if self.speed_sensor:
            current_speed = self.speed_sensor.get_speed()
            current_acceleration = self.speed_sensor.get_acceleration()
            # 这里可以添加日志记录
            
        return smooth_throttle

    def reset(self):
        """重置控制器状态"""
        self.pid.reset()
        self.pid.setpoint = 0.0

    def tune_pid(self, kp=None, ki=None, kd=None):
        """调整PID参数
        
        Args:
            kp (float, optional): 比例系数，影响响应速度
            ki (float, optional): 积分系数，消除静态误差
            kd (float, optional): 微分系数，抑制突变
        """
        if kp is not None:
            self.pid.Kp = kp
        if ki is not None:
            self.pid.Ki = ki
        if kd is not None:
            self.pid.Kd = kd 