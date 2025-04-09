import rclpy  # 导入 ROS2 Python 客户端库
from rclpy.node import Node  # 从 rclpy.node 模块中导入 Node 类
from std_msgs.msg import String  # 导入标准消息类型 String
import time

# 定义一个名为 MinimalPublisher 的类，继承自 Node
class MinimalPublisher(Node):
    def __init__(self):
        # 调用父类 Node 的构造函数，初始化节点名称为 'minimal_publisher'
        super().__init__('minimal_publisher')
        
        # 创建一个发布者，发布 String 类型的消息到 'topic' 主题，队列大小为 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # 初始化一个计数器变量
        self.i = 0

    def publish_messages(self):
        # 每秒发送100条消息
        for _ in range(100):
            msg = String()
            msg.data = 'Message %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1
            time.sleep(0.01)  # 每条消息间隔0.01秒

# 定义主函数
def main(args=None):
    # 初始化 rclpy 库
    rclpy.init(args=args)
    
    # 创建 MinimalPublisher 节点实例
    minimal_publisher = MinimalPublisher()
    
    try:
        while True:
            minimal_publisher.publish_messages()
    except KeyboardInterrupt:
        pass
    
    # 销毁节点
    minimal_publisher.destroy_node()
    
    # 关闭 rclpy 库
    rclpy.shutdown()

# 如果此脚本是主程序，则调用 main 函数
if __name__ == '__main__':
    main() 