#!/usr/bin/env python3
"""
ROS 2 交互式服务客户端节点示例
演示如何创建一个交互式客户端，让用户输入数字进行加法计算

这个示例展示了：
1. 如何创建交互式服务客户端
2. 如何处理用户输入
3. 如何优雅地处理服务调用
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class InteractiveAddTwoIntsClient(Node):
    """
    交互式加法计算服务客户端节点
    
    功能：
    - 提供交互式界面，让用户输入两个数字
    - 调用加法计算服务
    - 显示计算结果
    - 支持连续计算，直到用户退出
    """
    
    def __init__(self):
        super().__init__('interactive_add_two_ints_client')
        
        # 创建服务客户端
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 等待服务端可用
        self.get_logger().info('正在连接服务端...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务端启动...')
        
        self.get_logger().info('✅ 服务端已连接！')
        self.get_logger().info('🎯 现在可以开始计算加法了')
    
    def get_user_input(self):
        """
        获取用户输入的两个数字
        
        返回值：
        - tuple: (a, b) 用户输入的两个整数，如果用户想退出则返回 None
        """
        try:
            print("\n" + "="*50)
            print("🧮 加法计算器")
            print("="*50)
            
            # 获取第一个数字
            a_str = input("请输入第一个数字: ").strip()
            
            # 检查是否退出
            if a_str.lower() in ['q', 'quit', 'exit', '退出']:
                return None
            
            # 获取第二个数字
            b_str = input("请输入第二个数字: ").strip()
            
            # 检查是否退出
            if b_str.lower() in ['q', 'quit', 'exit', '退出']:
                return None
            
            # 转换为整数
            a = int(a_str)
            b = int(b_str)
            
            return (a, b)
            
        except ValueError:
            print("❌ 输入错误！请输入有效的整数。")
            return self.get_user_input()  # 递归重试
        except KeyboardInterrupt:
            print("\n👋 用户中断，退出程序")
            return None
    
    def send_request(self, a, b):
        """
        发送服务请求
        
        参数：
        - a: 第一个整数
        - b: 第二个整数
        
        返回值：
        - response: 服务响应对象，包含计算结果
        """
        # 创建请求对象
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # 发送请求并等待响应
        future = self.client.call_async(request)
        
        # 等待响应完成
        rclpy.spin_until_future_complete(self, future)
        
        # 获取响应结果
        response = future.result()
        
        return response
    
    def run_interactive_loop(self):
        """
        运行交互式循环
        """
        print("\n📋 使用说明：")
        print("- 输入两个数字进行加法计算")
        print("- 输入 'q' 或 'quit' 退出程序")
        print("- 按 Ctrl+C 也可以退出程序")
        
        while True:
            # 获取用户输入
            user_input = self.get_user_input()
            
            # 检查是否退出
            if user_input is None:
                break
            
            a, b = user_input
            
            try:
                # 发送请求
                self.get_logger().info(f'正在计算: {a} + {b}')
                response = self.send_request(a, b)
                
                # 显示结果
                result = response.sum
                print(f"\n✅ 计算结果: {a} + {b} = {result}")
                
            except Exception as e:
                print(f"❌ 计算失败: {e}")
                self.get_logger().error(f'服务调用失败: {e}')


def main(args=None):
    """
    主函数
    """
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建交互式客户端节点
    interactive_client = InteractiveAddTwoIntsClient()
    
    try:
        # 运行交互式循环
        interactive_client.run_interactive_loop()
        
    except KeyboardInterrupt:
        print("\n👋 程序被用户中断")
    except Exception as e:
        print(f"❌ 程序出错: {e}")
    finally:
        # 清理资源
        interactive_client.destroy_node()
        rclpy.shutdown()
        print("🔚 程序已退出")


if __name__ == '__main__':
    main() 