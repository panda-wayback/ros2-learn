#!/usr/bin/env python3
import math
import time
from simple_pid import PID
from .base_controller import BaseController

class DirectionController(BaseController):
    """方向控制器，使用PID控制转向"""
    def __init__(self, gyro_sensor=None):
        super().__init__('direction_controller')
        self.gyro_sensor = gyro_sensor
        
        # 创建 PID 控制器
        self.pid = PID(
            Kp=1.0,   # 比例系数
            Ki=0.1,   # 积分系数
            Kd=0.3,   # 微分系数
            setpoint=0.0,  # 初始目标角度
            output_limits=(-math.pi/2, math.pi/2),  # 限制输出范围
            sample_time=0.1,  # 采样时间
            auto_mode=True  # 自动模式
        )
        
        # 设置积分项限制，防止积分饱和
        self.pid.Ki_limits = (-1.0, 1.0)

    def compute_steering(self, target_angle, current_angle, current_angular_velocity):
        """计算转向控制输出
        
        Args:
            target_angle (float): 目标角度（弧度）
            current_angle (float): 当前角度（弧度）
            current_angular_velocity (float): 当前角速度（弧度/秒）
            
        Returns:
            float: 转向控制输出（角速度，弧度/秒）
        """
        if not self.is_enabled():
            return 0.0

        # 计算角度误差（考虑角度的循环性）
        angle_error = self.normalize_angle_diff(target_angle - current_angle)
        
        # 使用 PID 控制器计算转向输出
        steering = self.pid(current_angle + angle_error)
        
        # 考虑当前角速度进行阻尼
        damping_factor = 0.2
        steering -= current_angular_velocity * damping_factor
        
        return steering

    def reset(self):
        """重置控制器状态"""
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

    @staticmethod
    def normalize_angle_diff(angle_diff):
        """标准化角度差，使其在 [-pi, pi] 范围内
        
        Args:
            angle_diff (float): 角度差（弧度）
            
        Returns:
            float: 标准化后的角度差（弧度）
        """
        while angle_diff > math.pi:
            angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2.0 * math.pi
        return angle_diff

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
        self.pid.setpoint = self.target_angle

    def _normalize_error(self, error):
        """归一化误差到 -pi 到 pi 之间"""
        return ((error + math.pi) % (2 * math.pi)) - math.pi

    def reset_pid(self):
        """重置PID控制器状态"""
        self.pid.reset()
        self.pid.setpoint = self.target_angle

    def update(self, dt=0.1):
        """使用PID控制更新方向"""
        if not self.is_enabled():
            return 0.0

        # 获取当前状态
        current_angle = self.gyro_sensor.get_angle()
        current_angular_velocity = self.gyro_sensor.get_angular_velocity()

        # 使用 PID 控制器计算转向量
        # 将当前角速度作为微分项的反馈
        self.pid.Kd = -0.5  # 负号是因为我们想要抵消当前的角速度
        steering = self.pid(current_angle, dt=dt)
        
        # 更新陀螺仪
        self.gyro_sensor.update(steering, dt)
        
        return steering

    def reset(self):
        """重置方向控制器状态"""
        self.target_angle = 0.0
        self.reset_pid()
        self.gyro_sensor.reset()

    def tune_pid(self, kp=None, ki=None, kd=None):
        """调整PID参数"""
        if kp is not None:
            self.pid.Kp = kp
        if ki is not None:
            self.pid.Ki = ki
        if kd is not None:
            self.pid.Kd = kd 