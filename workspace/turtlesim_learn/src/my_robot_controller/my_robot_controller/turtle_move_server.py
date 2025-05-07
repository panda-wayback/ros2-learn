#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import math
from turtlesim.srv import TeleportAbsolute
from my_robot_controller.action import TurtleMove

class TurtleMoveServer(Node):
    def __init__(self):
        super().__init__('turtle_move_server')
        self._action_server = ActionServer(
            self,
            TurtleMove,
            'turtle_move',
            self.execute_callback)
        self.get_logger().info('乌龟移动服务已启动')
        
        # 创建客户端用于控制乌龟
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待乌龟服务...')

    def execute_callback(self, goal_handle):
        self.get_logger().info('收到移动请求')
        
        # 获取目标位置
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        target_theta = goal_handle.request.target_theta
        
        # 设置反馈信息
        feedback_msg = TurtleMove.Feedback()
        
        # 模拟移动过程
        current_x = 0.0
        current_y = 0.0
        step = 0.1  # 移动步长
        
        while True:
            # 计算当前位置到目标的距离
            distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            if distance < step:
                # 到达目标
                break
                
            # 更新当前位置
            current_x += step * (target_x - current_x) / distance
            current_y += step * (target_y - current_y) / distance
            
            # 发送反馈
            feedback_msg.current_x = current_x
            feedback_msg.current_y = current_y
            feedback_msg.distance_to_goal = distance
            goal_handle.publish_feedback(feedback_msg)
            
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('移动被取消')
                return TurtleMove.Result()
            
            # 短暂延迟
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 到达目标位置
        req = TeleportAbsolute.Request()
        req.x = target_x
        req.y = target_y
        req.theta = target_theta
        self.teleport_client.call_async(req)
        
        # 设置结果
        result = TurtleMove.Result()
        result.final_x = target_x
        result.final_y = target_y
        result.final_theta = target_theta
        result.success = True
        
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMoveServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 