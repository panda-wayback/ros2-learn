#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys
from my_robot_controller.action import TurtleMove

class TurtleMoveClient(Node):
    def __init__(self):
        super().__init__('turtle_move_client')
        self._action_client = ActionClient(self, TurtleMove, 'turtle_move')
        self.get_logger().info('乌龟移动客户端已启动')

    def send_goal(self, x, y, theta):
        self.get_logger().info(f'发送移动目标: x={x}, y={y}, theta={theta}')
        
        goal_msg = TurtleMove.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        goal_msg.target_theta = theta
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return
            
        self.get_logger().info('目标被接受')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'移动完成: 最终位置 x={result.final_x}, y={result.final_y}, theta={result.final_theta}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'收到反馈: 当前位置 x={feedback_msg.current_x}, y={feedback_msg.current_y}, 距离目标={feedback_msg.distance_to_goal}')

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 4:
        print("用法: python3 turtle_move_client.py x y theta")
        return
        
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3])
    
    client = TurtleMoveClient()
    client.send_goal(x, y, theta)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('移动被用户取消')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 