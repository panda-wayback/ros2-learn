#!/usr/bin/env python3

# 导入必要的ROS2 Python客户端库
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import json

class LightControlNode(Node):
    """
    灯光控制节点
    提供统一的灯光控制服务，支持：
    - 开关控制 (power)
    - 亮度控制 (brightness)
    - 颜色控制 (color)
    """
    def __init__(self):
        # 调用父类构造函数，设置节点名称为'light_control'
        super().__init__('light_control')
        
        # 创建服务
        self.service = self.create_service(
            Trigger,
            'light_control',
            self.control_callback
        )
        
        # 打印服务就绪信息
        self.get_logger().info('灯光控制服务已就绪!')

    def handle_power(self, value):
        """处理开关控制命令"""
        # 尝试将value转换为布尔值
        if value.lower() in ['true', '1', '开灯', 'on']:
            return True, "灯已打开"
        elif value.lower() in ['false', '0', '关灯', 'off']:
            return True, "灯已关闭"
        else:
            return False, f"未知的开关命令: {value}"

    def handle_brightness(self, value):
        """处理亮度控制命令"""
        try:
            # 尝试将value转换为数字
            level = float(value)
            if 0 <= level <= 100:
                return True, f"亮度已调节到 {level}%"
            else:
                return False, "亮度值必须在0-100之间"
        except ValueError:
            return False, "亮度调节失败，请输入0-100之间的数字"

    def handle_color(self, value):
        """处理颜色控制命令"""
        colors = {
            "红色": "已切换到红色",
            "绿色": "已切换到绿色",
            "蓝色": "已切换到蓝色",
            "白色": "已切换到白色"
        }
        if value in colors:
            return True, colors[value]
        else:
            return False, f"未知的颜色: {value}"

    def control_callback(self, request, response):
        """处理灯光控制命令"""
        try:
            # 从请求中获取命令
            command = json.loads(request.message)
            action = command.get('action')
            value = command.get('value')
            
            self.get_logger().info(f'收到命令: {action} = {value}')
            
            # 根据动作类型调用对应的处理函数
            if action == "power":
                success, message = self.handle_power(value)
            elif action == "brightness":
                success, message = self.handle_brightness(value)
            elif action == "color":
                success, message = self.handle_color(value)
            else:
                success = False
                message = f"未知的动作类型: {action}"
            
            # 设置响应
            response.success = success
            response.message = message
            return response
            
        except json.JSONDecodeError:
            response.success = False
            response.message = "无效的命令格式，请使用JSON格式"
            return response

def main(args=None):
    """
    主函数
    初始化ROS2，创建并运行灯光控制节点
    """
    try:
        # 初始化ROS2
        rclpy.init(args=args)
        # 创建灯光控制节点实例
        light_node = LightControlNode()
        # 运行节点，保持节点存活直到被终止
        rclpy.spin(light_node)
    except KeyboardInterrupt:
        print('\n接收到 Ctrl+C，正在关闭节点...')
    except Exception as e:
        print(f'\n发生错误: {str(e)}')
    finally:
        # 销毁节点
        if 'light_node' in locals():
            light_node.destroy_node()
        # 检查ROS2是否已经初始化，如果已初始化则关闭
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 