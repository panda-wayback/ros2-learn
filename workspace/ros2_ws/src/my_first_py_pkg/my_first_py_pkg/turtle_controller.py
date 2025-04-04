#!/usr/bin/env python3
import rclpy
import time
from .turtle_base import TurtleBase

class TurtleController(TurtleBase):
    def __init__(self):
        super().__init__('turtle_controller')
        # 创建一个定时器，用于自动控制
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 控制状态
        self.auto_mode = True
        self.current_action = 'accelerate'
        self.action_start_time = time.time()
        self.action_duration = 3.0  # 每个动作持续时间（秒）

    def timer_callback(self):
        """定时器回调函数，实现自动控制"""
        if not self.auto_mode:
            return

        current_time = time.time()
        elapsed_time = current_time - self.action_start_time

        if elapsed_time >= self.action_duration:
            # 切换下一个动作
            if self.current_action == 'accelerate':
                self.current_action = 'brake'
            elif self.current_action == 'brake':
                self.current_action = 'accelerate'
            self.action_start_time = current_time

        # 执行当前动作
        if self.current_action == 'accelerate':
            # 逐渐增加油门
            throttle_value = min(1.0, elapsed_time / self.action_duration)
            self.set_throttle(throttle_value)
            self.set_steering(0.0)  # 直行
        elif self.current_action == 'brake':
            # 逐渐增加刹车
            brake_value = min(1.0, elapsed_time / self.action_duration)
            self.set_brake(brake_value)
            self.set_steering(0.0)  # 直行

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()  # 确保在退出时停止乌龟
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 