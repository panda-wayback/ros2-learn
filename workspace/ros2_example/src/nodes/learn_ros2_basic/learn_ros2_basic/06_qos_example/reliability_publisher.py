#!/usr/bin/env python3
"""
QoS 可靠性(Reliability)演示 - 发布者节点

本节点只专注于演示QoS中“可靠性”策略的区别。
它会创建两个发布者，用两种不同的可靠性策略，在短时间内发布大量消息，
以模拟网络拥堵或处理能力不足的场景，从而让QoS的效果显现出来。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import time

class ReliabilityPublisher(Node):
    def __init__(self):
        super().__init__('reliability_publisher')

        # --- QoS策略1: 可靠 (RELIABLE) ---
        # 作用：保证每一条消息都必须成功送达。如果失败，会重试。
        # 结果：不会丢消息，但延迟可能较高。
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100  # 确保能缓存所有要发送的消息
        )
        self.reliable_pub = self.create_publisher(
            String, 
            'reliable_topic', 
            qos_profile=reliable_qos
        )
        self.get_logger().info("创建 'reliable_topic' 发布者 (QoS: 可靠)")

        # --- QoS策略2: 尽力而为 (BEST_EFFORT) ---
        # 作用：只管发，不保证送达，也不会重试。
        # 结果：延迟最低，但消息可能会丢失。
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # 对于尽力而为模式，通常只关心最新的消息
        )
        self.best_effort_pub = self.create_publisher(
            String,
            'best_effort_topic',
            qos_profile=best_effort_qos
        )
        self.get_logger().info("创建 'best_effort_topic' 发布者 (QoS: 尽力而为)")

    def publish_burst_messages(self, num_messages=100, duration_sec=1.0):
        """
        在指定时间内，爆发式地发布大量消息。
        """
        self.get_logger().info(
            f"\n将在 {duration_sec} 秒内，向两个Topic各发布 {num_messages} 条消息..."
        )
        
        # 计算每条消息之间的时间间隔
        sleep_interval = duration_sec / num_messages
        
        for i in range(num_messages):
            msg_text = f"Message #{i + 1}"
            msg = String(data=msg_text)
            
            # 同时向两个Topic发布
            self.reliable_pub.publish(msg)
            self.best_effort_pub.publish(msg)
            
            # 等待一小段时间
            time.sleep(sleep_interval)

        self.get_logger().info(f"\n🎉 {num_messages} 条消息已全部发出。")

def main(args=None):
    rclpy.init(args=args)
    node = ReliabilityPublisher()
    
    # 等待1秒，确保订阅者有时间连接
    time.sleep(1)
    
    # 执行发布逻辑
    node.publish_burst_messages()
    
    # 发布完成后再等待2秒，确保网络有时间传输最后的消息
    time.sleep(2)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 