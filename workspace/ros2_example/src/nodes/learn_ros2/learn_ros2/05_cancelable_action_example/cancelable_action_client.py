#!/usr/bin/env python3
"""
最简单的可取消Action客户端 - 2秒自动取消（单线程）
"""

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from example_interfaces.action import Fibonacci


class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_cancelable_client')
        
        self._action_client = ActionClient(self, Fibonacci, 'simple_action')
        self._goal_handle = None
        self._cancel_timer = None
        self._done = False
        
        print("🟢 简单客户端启动")
    
    def send_goal(self):
        """发送目标"""
        # 等待服务端
        self._action_client.wait_for_server()
        
        # 发送虚假目标
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        
        print("🚀 发送目标")
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_accepted_callback)
        
        print("💡 2秒后自动发送取消请求")
        
        # 创建2秒定时器（单次）
        self._cancel_timer = self.create_timer(2.0, self.auto_cancel)
    
    def goal_accepted_callback(self, future):
        """目标接受回调"""
        self._goal_handle = future.result()
        
        if self._goal_handle.accepted:
            print("✅ 目标被接受")
            # 获取结果
            result_future = self._goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
        else:
            print("❌ 目标被拒绝")
            self._done = True
    
    def result_callback(self, future):
        """结果回调"""
        result = future.result()
        print(f"🎯 最终结果: {result.result.sequence}")
        self._done = True
    
    def auto_cancel(self):
        """2秒后自动取消"""
        if self._cancel_timer:
            self._cancel_timer.destroy()  # 销毁定时器
            
        print("\n⏰ 2秒到了，发送取消请求...")
        
        if self._goal_handle and not self._done:
            # 发送取消请求
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_callback)
            print("📤 取消请求已发送")
        else:
            print("❌ 没有有效的目标句柄")
    
    def cancel_callback(self, future):
        """取消响应回调"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            print("📮 取消被接受")
        else:
            print("⚠️ 取消被拒绝")
    
    def is_done(self):
        return self._done


def main():
    rclpy.init()
    client = SimpleClient()
    
    try:
        client.send_goal()
        
        # 简单的spin循环
        while not client.is_done():
            rclpy.spin_once(client, timeout_sec=0.1)
            
        print("👋 客户端退出")
    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
