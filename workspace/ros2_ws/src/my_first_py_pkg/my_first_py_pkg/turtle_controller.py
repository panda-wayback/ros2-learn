#!/usr/bin/env python3
import rclpy
from .turtle_base import TurtleBase
from .controllers import DirectionController, MotionController, EnvironmentDetector

class TurtleController(TurtleBase):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # 初始化各个控制器
        self.direction_ctrl = DirectionController()
        self.motion_ctrl = MotionController(max_speed=0.5)
        self.env_detector = EnvironmentDetector(wall_hit_threshold=0.1)
        
        # 创建定时器
        self.update_rate = 0.1  # 10Hz
        self.timer = self.create_timer(self.update_rate, self.timer_callback)
        self.auto_mode = True

    def timer_callback(self):
        """定时器回调函数"""
        if not self.auto_mode:
            return

        # 环境检测
        if self.env_detector.check_collision(self.pose.x, self.pose.y):
            self.get_logger().info('检测到撞墙，改变方向')
            # 改变方向（在撞墙时交替左转和右转）
            if self.direction_ctrl.get_current_angle() >= 0:
                self.direction_ctrl.turn_right()
            else:
                self.direction_ctrl.turn_left()
            # 重置动作控制器
            self.motion_ctrl.reset()

        # 更新控制器状态
        motion_state = self.motion_ctrl.update(self.update_rate)
        steering_angle = self.direction_ctrl.update(self.update_rate)
        
        # 应用控制
        self.set_throttle(motion_state['throttle'])
        self.set_brake(motion_state['brake'])
        self.set_steering(steering_angle)

    def stop(self):
        """停止小乌龟"""
        self.motion_ctrl.brake_ctrl.set_brake_force(1.0)
        self.set_brake(1.0)
        self.set_throttle(0.0)

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