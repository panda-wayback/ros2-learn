#!/usr/bin/env python3
"""
最简单的可取消Action服务端 - 只为了演示取消机制
"""

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor  # 关键：导入多线程执行器
from example_interfaces.action import Fibonacci
import time


class SimpleServer(Node):
    def __init__(self):
        super().__init__('simple_cancelable_server')
        
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'simple_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        print("🟢 简单服务端启动")
    
    def goal_callback(self, goal_request):
        """目标接收回调"""
        print("📨 收到新目标")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """取消回调 - 关键！"""
        print("🚨 服务端收到取消请求！")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """执行回调 - 虚假进度条"""
        print("🚀 开始执行虚假任务")
        
        feedback = Fibonacci.Feedback()
        result = Fibonacci.Result()
        
        # 虚假进度条：运行20步，每步1秒
        for i in range(20):
            # 检查取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.sequence = [i]  # 返回当前步数
                print(f"❌ 任务被取消，在第{i}步")
                return result
            
            # 虚假进度
            feedback.sequence = [i]
            goal_handle.publish_feedback(feedback)
            print(f"📊 虚假进度: {i}/20")
            
            time.sleep(1)  # 每步1秒
        
        # 完成
        goal_handle.succeed()
        result.sequence = [20]
        print("🎉 任务完成")
        return result


def main():
    rclpy.init()
    server = SimpleServer()
    
    # 关键修复：使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    
    try:
        print("⚙️  服务端使用多线程执行器运行...")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
