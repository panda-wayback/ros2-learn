#!/usr/bin/env python3

# 导入必要的ROS2 Python客户端库
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 导入标准消息类型String

class PublisherNode(Node):
    """
    发布者节点类
    继承自ROS2的Node基类，用于创建一个发布者节点
    """
    # 定义话题名称常量
    TOPIC_NAME = 'chat_topic'
    NODE_NAME = 'publisher_node'
    
    def __init__(self):
        # 调用父类构造函数，设置节点名称
        super().__init__(self.NODE_NAME)
        # 创建一个发布者，发布String类型的消息到话题
        # 队列大小设置为10，用于缓存消息
        self.publisher_ = self.create_publisher(String, self.TOPIC_NAME, 10)
        # 设置定时器周期为1秒
        timer_period = 1.0  # seconds
        # 创建定时器，周期性地调用timer_callback函数
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # 初始化计数器
        self.i = 0

    def timer_callback(self):
        """
        定时器回调函数
        每秒执行一次，用于发布消息
        """
        # 创建String类型的消息对象
        msg = String()
        # 设置消息内容，包含计数器的值
        msg.data = 'Hello World: %d' % self.i
        # 发布消息到话题
        self.publisher_.publish(msg)
        # 在日志中打印发布的消息内容
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # 计数器加1
        self.i += 1

def main(args=None):
    """
    主函数
    初始化ROS2，创建并运行发布者节点
    """
    try:
        # 初始化ROS2
        rclpy.init(args=args)
        # 创建发布者节点实例
        publisher_node = PublisherNode()
        # 运行节点，保持节点存活直到被终止
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        print('\n接收到 Ctrl+C，正在关闭节点...')
    except Exception as e:
        print(f'\n发生错误: {str(e)}')
    finally:
        # 销毁节点
        if 'publisher_node' in locals():
            publisher_node.destroy_node()
        # 检查ROS2是否已经初始化，如果已初始化则关闭
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 