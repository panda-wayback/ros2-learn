#!/usr/bin/env python3
"""
QoS 可靠性(Reliability)演示 - 订阅者节点

本节点只专注于演示QoS中“可靠性”策略的区别。
它会同时订阅“可靠”和“尽力而为”两个Topic，并统计各自收到的消息总数，
通过对比最终的数量，可以清晰地看出不同QoS策略带来的巨大差异。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class ReliabilitySubscriber(Node):
    def __init__(self):
        super().__init__('reliability_subscriber')

        # 初始化消息计数器
        self.reliable_msg_count = 0
        self.best_effort_msg_count = 0

        # --- QoS策略1: 可靠 (RELIABLE) ---
        # 必须使用与发布者兼容的QoS设置才能成功连接
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        self.reliable_sub = self.create_subscription(
            String,
            'reliable_topic',
            self.reliable_callback,
            qos_profile=reliable_qos
        )
        self.get_logger().info("创建 'reliable_topic' 订阅者 (QoS: 可靠)")

        # --- QoS策略2: 尽力而为 (BEST_EFFORT) ---
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.best_effort_sub = self.create_subscription(
            String,
            'best_effort_topic',
            self.best_effort_callback,
            qos_profile=best_effort_qos
        )
        self.get_logger().info("创建 'best_effort_topic' 订阅者 (QoS: 尽力而为)")
        
        self.get_logger().info("\n--- 订阅者已准备就绪，开始接收消息... ---")
        
        # 创建一个定时器，用于在一段时间后打印最终统计结果
        self.summary_timer = self.create_timer(4.0, self.print_summary)

    def reliable_callback(self, msg):
        """处理“可靠”消息的回调"""
        self.reliable_msg_count += 1
        # 为了避免刷屏，我们只打印部分消息
        if self.reliable_msg_count % 10 == 0 or self.reliable_msg_count <= 5:
            self.get_logger().info(f"🚀 [可靠] 收到: '{msg.data}' (总计: {self.reliable_msg_count})")

    def best_effort_callback(self, msg):
        """处理“尽力而为”消息的回调"""
        self.best_effort_msg_count += 1
        self.get_logger().info(f"🌊 [尽力] 收到: '{msg.data}' (总计: {self.best_effort_msg_count})")

    def print_summary(self):
        """打印最终的统计结果"""
        self.get_logger().info("\n\n--- 最终统计结果 ---")
        self.get_logger().info(f"✅ 可靠 (RELIABLE) Topic:    共收到 {self.reliable_msg_count} / 100 条消息")
        self.get_logger().info(f"❌ 尽力而为 (BEST_EFFORT) Topic: 共收到 {self.best_effort_msg_count} / 100 条消息")
        self.get_logger().info("----------------------")
        self.get_logger().info("结论：在消息发送速率超过系统处理能力时，RELIABLE保证了消息的完整性，而BEST_EFFORT则为了实时性牺牲了大量消息。")
        
        # 打印完总结后，关闭节点
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ReliabilitySubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # 这是节点被定时器自动关闭时产生的正常异常
        pass
    finally:
        # 在spin结束后，节点可能已经被销毁了，这里加一个检查
        if rclpy.ok() and node.handle:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 