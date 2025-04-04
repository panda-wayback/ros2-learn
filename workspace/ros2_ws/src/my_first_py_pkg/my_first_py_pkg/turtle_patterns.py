#!/usr/bin/env python3
import time

class TurtlePatterns:
    def __init__(self, turtle_base):
        self.turtle = turtle_base
        self.patterns = {
            'square': self.square_pattern,
            'circle': self.circle_pattern,
            'figure8': self.figure8_pattern
        }

    def square_pattern(self, side_length=2.0, speed=2.0):
        """让乌龟画正方形"""
        for _ in range(4):
            # 前进
            self.turtle.move_forward(speed)
            time.sleep(side_length / speed)
            # 右转90度
            self.turtle.turn_right(1.57)  # 90度约等于1.57弧度
            time.sleep(1.0)

    def circle_pattern(self, radius=2.0, speed=2.0):
        """让乌龟画圆形"""
        # 设置角速度使乌龟做圆周运动
        angular_speed = speed / radius
        self.turtle.move_forward(speed)
        self.turtle.turn_right(angular_speed)
        time.sleep(2 * 3.14159 * radius / speed)  # 2πr/v 是绕一圈的时间

    def figure8_pattern(self, size=2.0, speed=2.0):
        """让乌龟画8字形"""
        # 画第一个圆
        self.circle_pattern(size, speed)
        # 画第二个圆（反向）
        self.turtle.turn_left(2 * speed / size)  # 反向旋转
        time.sleep(2 * 3.14159 * size / speed)

    def run_pattern(self, pattern_name, **kwargs):
        """运行指定的运动模式"""
        if pattern_name in self.patterns:
            self.patterns[pattern_name](**kwargs)
        else:
            self.turtle.get_logger().error(f'未知的运动模式: {pattern_name}') 