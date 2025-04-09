import rclpy  # 导入 ROS2 Python 客户端库
from rclpy.node import Node  # 从 rclpy.node 模块中导入 Node 类
from std_msgs.msg import String  # 导入标准消息类型 String
import time

# 定义一个名为 MinimalSubscriber 的类，继承自 Node
class MinimalSubscriber(Node):
    def __init__(self):
        # 调用父类 Node 的构造函数，初始化节点名称为 'minimal_subscriber'
        super().__init__('minimal_subscriber')
        
        # 创建一个订阅者，订阅 String 类型的消息，从 'topic' 主题接收消息，队列大小为 10
        self.subscription = self.create_subscription(
            String,  # 消息类型
            'topic',  # 主题名称
            self.listener_callback,  # 回调函数
            10)  # 队列大小
        
        # 防止未使用变量警告
        self.subscription  
        self.last_message_number = -1

    # 定义订阅者的回调函数
    def listener_callback(self, msg):
        # 模拟处理消息需要时间
        time.sleep(0.1)  # 每条消息处理需要0.1秒
        current_number = int(msg.data.split()[-1])
        
        # 检查是否有消息丢失
        if self.last_message_number != -1 and current_number != self.last_message_number + 1:
            self.get_logger().warn('Messages lost! Last: %d, Current: %d' % 
                                 (self.last_message_number, current_number))
        
        self.last_message_number = current_number
        self.get_logger().info('Processing: "%s"' % msg.data)

# 定义主函数
def main(args=None):
    # 初始化 rclpy 库
    rclpy.init(args=args)
    
    # 创建 MinimalSubscriber 节点实例
    minimal_subscriber = MinimalSubscriber()
    
    # 让节点保持运行，等待回调函数被调用
    rclpy.spin(minimal_subscriber)
    
    # 销毁节点
    minimal_subscriber.destroy_node()
    
    # 关闭 rclpy 库
    rclpy.shutdown()

# 如果此脚本是主程序，则调用 main 函数
if __name__ == '__main__':
    main() 