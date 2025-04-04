#!/usr/bin/env python3
import math
from .base_controller import BaseController

class DirectionController(BaseController):
    """方向控制器"""
    def __init__(self, gyro_sensor):
        super().__init__('direction_controller')
        self.gyro_sensor = gyro_sensor
        self.target_angle = 0.0   # 目标角度
        self.max_turn_rate = math.pi / 2  # 最大转向速率（弧度/秒）
        
        # PID控制参数
        self.kp = 2.0   # 比例系数
        self.ki = 0.1   # 积分系数
        self.kd = 0.5   # 微分系数
        
        # PID控制状态
        self.error_sum = 0.0
        self.last_error = 0.0
        self.max_error_sum = math.pi  # 积分项限制

    def turn_left(self):
        """左转90度"""
        current_angle = self.gyro_sensor.get_angle()
        self.target_angle = current_angle + math.pi / 2
        self._normalize_angle()
        self.reset_pid()

    def turn_right(self):
        """右转90度"""
        current_angle = self.gyro_sensor.get_angle()
        self.target_angle = current_angle - math.pi / 2
        self._normalize_angle()
        self.reset_pid()

    def set_angle(self, angle):
        """设置目标角度"""
        self.target_angle = angle
        self._normalize_angle()
        self.reset_pid()

    def _normalize_angle(self):
        """将角度归一化到 -pi 到 pi 之间"""
        self.target_angle = ((self.target_angle + math.pi) % (2 * math.pi)) - math.pi

    def reset_pid(self):
        """重置PID控制器状态"""
        self.error_sum = 0.0
        self.last_error = 0.0

    def update(self, dt=0.1):
        """使用PID控制更新方向"""
        if not self.is_enabled():
            return 0.0

        # 获取当前状态
        current_angle = self.gyro_sensor.get_angle()
        current_angular_velocity = self.gyro_sensor.get_angular_velocity()

        # 计算角度误差
        error = self.target_angle - current_angle
        # 归一化误差到 -pi 到 pi 之间
        error = ((error + math.pi) % (2 * math.pi)) - math.pi
        
        # 更新积分项
        self.error_sum += error * dt
        # 防止积分饱和
        self.error_sum = min(max(-self.max_error_sum, self.error_sum), self.max_error_sum)
        
        # 计算微分项（使用陀螺仪的角速度）
        error_diff = -current_angular_velocity  # 负号是因为我们想要抵消当前的角速度
        
        # PID控制计算转向量
        steering = (
            self.kp * error +           # 比例项
            self.ki * self.error_sum +  # 积分项
            self.kd * error_diff        # 微分项（使用角速度）
        )
        
        # 限制转向速率
        steering = min(max(-self.max_turn_rate, steering), self.max_turn_rate)
        
        # 更新上一次误差
        self.last_error = error
        
        # 更新陀螺仪
        self.gyro_sensor.update(steering, dt)
        
        return steering

    def reset(self):
        """重置方向控制器状态"""
        self.target_angle = 0.0
        self.reset_pid()
        self.gyro_sensor.reset() 