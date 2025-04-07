#!/usr/bin/env python3
import math
from simple_pid import PID
from .base_controller import BaseController

class DirectionController(BaseController):
    """方向控制器
    
    功能：
    1. 接收转向力度（-1到1）
       - 负数：向左转
       - 正数：向右转
       - 绝对值：转向力度
    2. 使用陀螺仪数据和PID控制器平滑转向输出
    3. 输出角速度（弧度/秒）
    
    使用示例：
    ```python
    # 创建陀螺仪和控制器
    gyro = GyroSensor()
    controller = DirectionController(gyro)
    
    # 在控制循环中持续调用
    def control_loop():
        while True:
            # 获取转向输入（例如从游戏手柄）
            steering = get_steering_input()  # -1到1的值
            
            # 计算转向控制
            angular_velocity = controller.compute(steering)
            
            # 控制小车
            cmd = Twist()
            cmd.angular.z = angular_velocity
            cmd_vel_publisher.publish(cmd)
            
            # 等待固定时间
            time.sleep(0.01)  # 10ms
    ```
    """
    def __init__(self, gyro_sensor):
        super().__init__('direction_controller')
        self.gyro_sensor = gyro_sensor
        
        # PID控制器配置
        self.pid = PID(
            Kp=1.0,   # 比例系数
            Ki=0.1,   # 积分系数
            Kd=0.3,   # 微分系数
            setpoint=0.0,
            output_limits=(-math.pi/2, math.pi/2),  # 限制最大角速度
            sample_time=0.01,  # 固定采样时间10ms
            auto_mode=True
        )
        self.pid.Ki_limits = (-0.5, 0.5)  # 防止积分饱和

    def compute(self, steering):
        """计算转向控制输出
        
        Args:
            steering (float): 转向力度，范围[-1, 1]
                - 负数：向左转
                - 正数：向右转
                - 绝对值：转向力度
                
        Returns:
            float: 角速度输出（弧度/秒）
        """
        if not self.is_enabled():
            return 0.0
        
        # 限制输入范围
        steering = min(max(-1.0, steering), 1.0)  # 确保转向输入在[-1, 1]范围内
                                                    # -1: 最大左转
                                                    #  0: 直行
                                                    #  1: 最大右转
        
        # 获取当前状态
        current_angle = self.gyro_sensor.get_angle()  # 获取当前车身相对于初始方向的偏转角度（弧度）
        current_velocity = self.gyro_sensor.get_angular_velocity()  # 获取当前车身旋转的角速度（弧度/秒）
        
        # 计算目标角度（基于当前角度和转向力度）
        target_angle = current_angle + steering * math.pi/4  # 计算目标偏转角度
                                                            # steering * math.pi/4: 最大可偏转45度（π/4弧度）
                                                            # 正值表示向右偏转，负值表示向左偏转
        
        # 使用PID计算角速度
        angular_velocity = self._compute_pid(current_angle, target_angle, current_velocity)  # 通过PID控制器计算需要的角速度
                                                                                            # 输出单位为弧度/秒
                                                                                            # 正值表示顺时针旋转，负值表示逆时针旋转
        
        return angular_velocity

    def _compute_pid(self, current_angle, target_angle, current_velocity):
        """计算PID控制输出
        
        Args:
            current_angle (float): 当前角度
            target_angle (float): 目标角度
            current_velocity (float): 当前角速度
            
        Returns:
            float: PID控制输出
        """
        # 设置PID目标值
        self.pid.setpoint = target_angle
        
        # 计算基础角速度
        angular_velocity = self.pid(current_angle)
        
        # 根据当前角速度调整输出，防止突然转向
        if current_velocity * angular_velocity < 0:  # 如果转向方向相反
            angular_velocity *= 0.5  # 降低转向速度
            
        return angular_velocity

    def reset(self):
        """重置控制器状态"""
        self.pid.reset()
        self.pid.setpoint = 0.0

    def tune(self, kp=None, ki=None, kd=None):
        """调整PID参数
        
        Args:
            kp (float, optional): 比例系数
            ki (float, optional): 积分系数
            kd (float, optional): 微分系数
        """
        if kp is not None:
            self.pid.Kp = kp
        if ki is not None:
            self.pid.Ki = ki
        if kd is not None:
            self.pid.Kd = kd 