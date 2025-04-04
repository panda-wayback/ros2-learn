#!/usr/bin/env python3
import time
import math
from .base_sensor import BaseSensor

class GyroSensor(BaseSensor):
    """陀螺仪传感器"""
    def __init__(self):
        super().__init__('gyro_sensor')
        self._current_angle = 0.0      # 当前角度
        self._angular_velocity = 0.0    # 角速度
        self._last_update_time = time.time()
        self._max_angular_velocity = math.pi  # 最大角速度（弧度/秒）
        self._noise_factor = 0.01  # 模拟传感器噪声
        
        self._last_reading = {
            'angle': 0.0,              # 当前角度（弧度）
            'angular_velocity': 0.0,    # 角速度（弧度/秒）
            'timestamp': time.time()    # 时间戳
        }

    def update(self, steering_angle, dt=0.1):
        """更新陀螺仪状态"""
        if not self.is_enabled():
            return self._last_reading

        current_time = time.time()
        
        # 计算目标角速度（基于转向角和当前角度的差）
        angle_diff = steering_angle - self._current_angle
        # 归一化角度差到 -pi 到 pi 之间
        angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi
        
        # 模拟角速度变化（添加一些惯性和限制）
        target_angular_velocity = angle_diff / dt
        target_angular_velocity = min(max(target_angular_velocity, -self._max_angular_velocity), 
                                   self._max_angular_velocity)
        
        # 模拟角速度的渐变变化（添加惯性）
        angular_velocity_diff = target_angular_velocity - self._angular_velocity
        self._angular_velocity += angular_velocity_diff * 0.5  # 角速度响应系数
        
        # 添加一些随机噪声
        noise = (random.random() - 0.5) * self._noise_factor
        self._angular_velocity += noise
        
        # 更新角度
        self._current_angle += self._angular_velocity * dt
        # 归一化角度
        self._current_angle = ((self._current_angle + math.pi) % (2 * math.pi)) - math.pi
        
        # 更新传感器读数
        self._last_reading = {
            'angle': self._current_angle,
            'angular_velocity': self._angular_velocity,
            'timestamp': current_time
        }
        
        return self._last_reading

    def get_angle(self):
        """获取当前角度"""
        return self._current_angle

    def get_angular_velocity(self):
        """获取当前角速度"""
        return self._angular_velocity

    def reset(self):
        """重置陀螺仪状态"""
        self._current_angle = 0.0
        self._angular_velocity = 0.0
        self._last_update_time = time.time()
        self._last_reading = {
            'angle': 0.0,
            'angular_velocity': 0.0,
            'timestamp': time.time()
        } 