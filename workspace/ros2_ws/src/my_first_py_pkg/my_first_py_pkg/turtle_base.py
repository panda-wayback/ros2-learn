#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBase(Node):
    def __init__(self, node_name='turtle_base'):
        super().__init__(node_name)
        # 创建一个发布者，发布到 /turtle1/cmd_vel 话题
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 速度控制参数
        self.max_speed = float(2.0)  # 最大速度
        self.max_angular_speed = float(1.0)  # 最大角速度
        self.brake_force = float(0.5)  # 刹车力度
        
        # 当前控制状态
        self.throttle = float(0.0)  # 油门值 0.0 ~ 1.0
        self.brake = float(0.0)     # 刹车值 0.0 ~ 1.0
        self.steering = float(0.0)  # 转向值 -1.0 ~ 1.0
        
        # 当前速度
        self.current_speed = float(0.0)
        self.current_angular_speed = float(0.0)
        
        # 创建定时器用于更新速度
        self.control_timer = self.create_timer(0.1, self.control_callback)  # 10Hz控制频率
        
        self.get_logger().info('乌龟基础控制器已启动！')

    def control_callback(self):
        """定时器回调函数，更新速度"""
        # 计算目标速度
        target_speed = float(self.throttle * self.max_speed)
        target_angular_speed = float(self.steering * self.max_angular_speed)
        
        # 应用刹车
        if self.brake > 0:
            target_speed = float(0.0)
            # 快速减速
            self.current_speed = float(max(0.0, self.current_speed - self.brake_force * self.brake))
        else:
            # 平滑加速/减速到目标速度
            if target_speed > self.current_speed:
                self.current_speed = float(min(target_speed, self.current_speed + 0.1))
            elif target_speed < self.current_speed:
                self.current_speed = float(max(target_speed, self.current_speed - 0.1))
        
        # 更新角速度
        self.current_angular_speed = float(target_angular_speed)
        
        # 发布速度命令
        msg = Twist()
        msg.linear.x = float(self.current_speed)
        msg.angular.z = float(self.current_angular_speed)
        self.publisher_.publish(msg)
        
        # 打印当前速度信息
        self.get_logger().info('速度: %.2f m/s, 角速度: %.2f rad/s' % 
                             (self.current_speed, self.current_angular_speed))

    def set_throttle(self, value):
        """设置油门值 (0.0 ~ 1.0)"""
        self.throttle = float(max(0.0, min(1.0, value)))
        self.brake = float(0.0)  # 踩油门时松开刹车

    def set_brake(self, value):
        """设置刹车值 (0.0 ~ 1.0)"""
        self.brake = float(max(0.0, min(1.0, value)))
        self.throttle = float(0.0)  # 踩刹车时松开油门

    def set_steering(self, value):
        """设置转向值 (-1.0 ~ 1.0)"""
        self.steering = float(max(-1.0, min(1.0, value)))

    def get_speed(self):
        """获取当前速度"""
        return float(self.current_speed)

    def get_angular_speed(self):
        """获取当前角速度"""
        return float(self.current_angular_speed)

    def move_forward(self, speed=2.0):
        """让乌龟前进，带加减速"""
        self.set_throttle(1.0)
        self.get_logger().info('前进: 目标速度=%.2f' % speed)

    def move_backward(self, speed=2.0):
        """让乌龟后退，带加减速"""
        self.set_throttle(-1.0)
        self.get_logger().info('后退: 目标速度=%.2f' % speed)

    def turn_left(self, speed=1.0):
        """让乌龟左转，带加减速"""
        self.set_steering(-1.0)
        self.get_logger().info('左转: 目标角速度=%.2f' % speed)

    def turn_right(self, speed=1.0):
        """让乌龟右转，带加减速"""
        self.set_steering(1.0)
        self.get_logger().info('右转: 目标角速度=%.2f' % speed)

    def stop(self):
        """停止乌龟，带减速"""
        self.set_throttle(0.0)
        self.get_logger().info('停止') 