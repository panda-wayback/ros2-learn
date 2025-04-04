#!/usr/bin/env python3
import time
from .base_sensor import BaseSensor

class SpeedSensor(BaseSensor):
    """速度传感器"""
    def __init__(self):
        super().__init__('speed_sensor')
        self._current_speed = 0.0
        self._acceleration = 0.0  # 当前加速度
        self._last_speed = 0.0
        self._last_update_time = time.time()

    def update(self, throttle, brake):
        """根据油门和刹车更新速度状态"""
        if not self.is_enabled():
            return self._current_speed

        current_time = time.time()
        dt = current_time - self._last_update_time

        # 计算加速度（考虑油门和刹车的影响）
        throttle_acc = throttle * 2.0  # 油门导致的加速度
        brake_dec = brake * 3.0       # 刹车导致的减速度
        friction_dec = 0.5            # 摩擦力导致的减速度
        
        # 合成加速度
        self._acceleration = throttle_acc - brake_dec
        if self._current_speed > 0:
            self._acceleration -= friction_dec
        elif self._current_speed < 0:
            self._acceleration += friction_dec

        # 更新速度
        self._last_speed = self._current_speed
        self._current_speed += self._acceleration * dt
        
        # 确保速度不会小于0
        self._current_speed = max(0.0, self._current_speed)
        
        # 更新时间
        self._last_update_time = current_time
        self._last_reading = {
            'speed': self._current_speed,
            'acceleration': self._acceleration
        }
        
        return self._current_speed

    def get_speed(self):
        """获取当前速度"""
        return self._current_speed

    def get_acceleration(self):
        """获取当前加速度"""
        return self._acceleration

    def reset(self):
        """重置速度传感器"""
        self._current_speed = 0.0
        self._acceleration = 0.0
        self._last_speed = 0.0
        self._last_update_time = time.time()
        self._last_reading = None 