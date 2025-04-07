#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_first_py_pkg.controllers.motion_controller import MotionController
from my_first_py_pkg.controllers.direction_controller import DirectionController
from my_first_py_pkg.controllers.throttle_controller import ThrottleController
from my_first_py_pkg.controllers.brake_controller import BrakeController
from my_first_py_pkg.sensors.speed_sensor import SpeedSensor
from my_first_py_pkg.sensors.gyro_sensor import GyroSensor

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller_node')
        
        # 创建发布者和订阅者
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 创建传感器（先创建传感器）
        self.speed_sensor = SpeedSensor()
        self.gyro_sensor = GyroSensor()
        
        # 创建控制器（后创建控制器，并传入需要的传感器）
        self.motion_controller = MotionController()
        self.direction_controller = DirectionController(gyro_sensor=self.gyro_sensor)
        self.throttle_controller = ThrottleController(speed_sensor=self.speed_sensor)  # 速度传感器现在是可选的，仅用于调试
        self.brake_controller = BrakeController(speed_sensor=self.speed_sensor)
        
        # 设置目标点（可以通过服务或参数动态修改）
        self.target_x = 8.0
        self.target_y = 8.0
        
        # 当前位置
        self.current_x = 5.544445  # 乌龟的初始x位置
        self.current_y = 5.544445  # 乌龟的初始y位置
        
        # 创建定时器，控制频率为10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('乌龟控制节点已启动')

    def pose_callback(self, msg: Pose):
        """接收乌龟位姿信息并更新传感器数据"""
        # 更新位置
        self.current_x = msg.x
        self.current_y = msg.y
        
        # 更新传感器数据
        self.speed_sensor.update(msg.linear_velocity, msg.angular_velocity)
        self.gyro_sensor.update(msg.theta, msg.angular_velocity)

    def control_loop(self):
        """主控制循环"""
        try:
            # 获取当前速度和角度
            current_speed = self.speed_sensor.get_speed()
            current_angle = self.gyro_sensor.get_angle()
            
            # 计算运动控制指令
            target_speed, target_angle = self.motion_controller.compute_motion(
                self.target_x, self.target_y,
                self.current_x, self.current_y,
                current_angle
            )
            
            # 方向控制
            angular_velocity = self.direction_controller.compute(target_angle)
            
            # 速度控制
            if target_speed > 0:
                # 将目标速度归一化为0~1的油门值
                normalized_throttle = min(target_speed / 2.0, 1.0)  # 假设最大速度为2.0
                # 计算平滑的油门输出
                linear_velocity = self.throttle_controller.compute(normalized_throttle)
                self.brake_controller.reset()  # 重置刹车状态
            else:
                # 使用刹车控制
                linear_velocity = self.brake_controller.update()
                self.throttle_controller.reset()  # 重置油门状态
            
            # 发布控制指令
            cmd_vel = Twist()
            cmd_vel.linear.x = float(linear_velocity)
            cmd_vel.angular.z = float(angular_velocity)
            self.cmd_vel_pub.publish(cmd_vel)
            
            # 记录调试信息
            self.get_logger().debug(
                f'位置: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'速度: {current_speed:.2f}, 角度: {current_angle:.2f}, '
                f'目标: ({self.target_x:.2f}, {self.target_y:.2f})'
            )
            
        except Exception as e:
            self.get_logger().error(f'控制循环出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 