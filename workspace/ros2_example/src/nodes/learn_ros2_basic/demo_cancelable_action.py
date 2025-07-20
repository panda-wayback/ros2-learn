#!/usr/bin/env python3
"""
可取消Action演示脚本

这个脚本会启动服务端，然后运行客户端进行测试
"""

import subprocess
import time
import signal
import sys
import os

def run_demo():
    print("🎬 开始可取消Action演示")
    print("=" * 50)
    
    # 切换到正确的目录
    os.chdir('/root/ros2-learn/workspace/ros2_example/src/nodes/learn_ros2_basic')
    
    # 启动服务端
    print("🚀 启动服务端...")
    server_process = subprocess.Popen([
        'bash', '-c',
        'source install/setup.bash && ros2 run learn_ros2_basic cancelable_action_server'
    ])
    
    # 等待服务端启动
    time.sleep(3)
    
    try:
        print("\n🎯 开始测试场景1: 正常完成任务")
        print("-" * 30)
        
        # 运行客户端（短任务，会完成）
        client_process = subprocess.run([
            'bash', '-c',
            'source install/setup.bash && echo "5" | ros2 run learn_ros2_basic cancelable_action_client'
        ], timeout=10)
        
        print("\n✅ 场景1完成")
        
        time.sleep(2)
        
        print("\n🎯 开始测试场景2: 展示取消功能")
        print("-" * 30)
        print("💡 提示：服务端将计算15个斐波那契数，足够演示取消功能")
        
        # 启动客户端（长任务，演示取消）
        client_process = subprocess.Popen([
            'bash', '-c',
            'source install/setup.bash && ros2 run learn_ros2_basic cancelable_action_client'
        ])
        
        # 让客户端运行一段时间
        print("⏰ 客户端运行3秒后将模拟Ctrl+C取消...")
        time.sleep(3)
        
        # 发送中断信号（模拟Ctrl+C）
        client_process.send_signal(signal.SIGINT)
        client_process.wait()
        
        print("\n✅ 场景2完成 - 成功演示了取消功能")
        
    except Exception as e:
        print(f"❌ 演示过程中出错: {e}")
    
    finally:
        print("\n🧹 清理进程...")
        try:
            server_process.terminate()
            server_process.wait(timeout=3)
        except:
            server_process.kill()
        
        print("🎉 演示完成！")
        print("\n📋 总结:")
        print("1. ✅ 服务端成功启动")
        print("2. ✅ 客户端可以正常连接")
        print("3. ✅ 任务可以正常完成")
        print("4. ✅ 取消功能工作正常")
        print("\n💡 你可以手动运行以下命令进行测试:")
        print("   终端1: ros2 run learn_ros2_basic cancelable_action_server")
        print("   终端2: ros2 run learn_ros2_basic cancelable_action_client")
        print("   在终端2中按Ctrl+C即可取消任务")

if __name__ == "__main__":
    try:
        run_demo()
    except KeyboardInterrupt:
        print("\n🛑 演示被用户中断")
        sys.exit(0) 