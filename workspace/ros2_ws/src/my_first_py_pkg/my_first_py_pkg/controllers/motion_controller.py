#!/usr/bin/env python3
import time
import math
from .base_controller import BaseController
from .throttle_controller import ThrottleController
from .brake_controller import BrakeController
from ..sensors.speed_sensor import SpeedSensor

class MotionController(BaseController):
    """动作控制器，协调油门和刹车"""
    def __init__(self, max_speed=0.5):
        super().__init__('motion_controller')
        # 控制器
        self.throttle_ctrl = ThrottleController(max_speed)
        self.brake_ctrl = BrakeController()
        # 传感器
        self.speed_sensor = SpeedSensor()
        
        # 动作状态
        self.current_action = 'accelerate'
        self.action_start_time = time.time()
        self.action_duration = 5.0

        self.min_distance = 0.1  # 到达目标点的最小距离
        self.max_speed = 2.0     # 最大速度
        self.min_speed = 0.5     # 最小速度
        self.slow_down_distance = 2.0  # 开始减速的距离

    def update(self, dt=0.1):
        """更新动作控制器状态"""
        if not self.is_enabled():
            return {'throttle': 0.0, 'brake': 0.0}

        current_time = time.time()
        elapsed_time = current_time - self.action_start_time

        # 切换动作
        if elapsed_time >= self.action_duration:
            self.current_action = 'brake' if self.current_action == 'accelerate' else 'accelerate'
            self.action_start_time = current_time
            elapsed_time = 0

        # 根据当前动作设置目标值
        if self.current_action == 'accelerate':
            self.throttle_ctrl.set_target_speed(self.throttle_ctrl.max_speed)
            self.brake_ctrl.set_brake_force(0.0)
        else:
            self.throttle_ctrl.set_target_speed(0.0)
            self.brake_ctrl.set_brake_force(1.0)

        # 获取当前速度
        current_speed = self.speed_sensor.get_speed()

        # 更新控制器状态
        throttle = self.throttle_ctrl.update(current_speed, dt)
        brake = self.brake_ctrl.update(dt)

        # 更新速度传感器
        self.speed_sensor.update(throttle, brake)

        return {'throttle': throttle, 'brake': brake}

    def get_speed(self):
        """获取当前速度"""
        return self.speed_sensor.get_speed()

    def get_acceleration(self):
        """获取当前加速度"""
        return self.speed_sensor.get_acceleration()

    def reset(self):
        """重置动作控制器状态"""
        self.throttle_ctrl.reset()
        self.brake_ctrl.reset()
        self.speed_sensor.reset()
        self.current_action = 'accelerate'
        self.action_start_time = time.time()

    def compute_motion(self, target_x, target_y, current_x, current_y, current_speed, current_angle):
        """计算运动控制指令
        
        Args:
            target_x (float): 目标点x坐标
            target_y (float): 目标点y坐标
            current_x (float): 当前x坐标
            current_y (float): 当前y坐标
            current_speed (float): 当前速度
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
        """根据距离计算目标速度"""
        if distance < self.min_distance:
            return 0.0
        
        # 在接近目标点时逐渐减速
        if distance < self.slow_down_distance:
            speed_factor = distance / self.slow_down_distance
            target_speed = self.min_speed + (self.max_speed - self.min_speed) * speed_factor
        else:
            target_speed = self.max_speed
            
        return target_speed 