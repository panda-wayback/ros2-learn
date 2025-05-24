#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareTurtle(Node):
    def __init__(self):
        super().__init__('square_turtle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Square turtle node has been started')
        
        # 初始化变量
        self.start_time = time.time()
        self.current_action = 'forward'  # 初始动作为前进
        self.side_length = 2.0  # 正方形边长（单位：米）
        self.angular_speed = 1.0  # 角速度（单位：弧度/秒）
        self.linear_speed = 0.5  # 线速度（单位：米/秒）
        
    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if self.current_action == 'forward':
            msg.linear.x = self.linear_speed
            if elapsed_time >= self.side_length / self.linear_speed:
                self.current_action = 'turn'
                self.start_time = current_time
        elif self.current_action == 'turn':
            msg.angular.z = self.angular_speed
            if elapsed_time >= 1.57 / self.angular_speed:  # 90度 = 1.57弧度
                self.current_action = 'forward'
                self.start_time = current_time
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    square_turtle = SquareTurtle()
    rclpy.spin(square_turtle)
    square_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 