#!/usr/bin/env python3

import subprocess
import json
import os

def get_workspace_packages():
    """获取当前工作空间中的包"""
    try:
        workspace_path = os.getcwd()
        src_path = os.path.join(workspace_path, 'src')
        packages = []
        for item in os.listdir(src_path):
            item_path = os.path.join(src_path, item)
            if os.path.isdir(item_path) and os.path.exists(os.path.join(item_path, 'package.xml')):
                packages.append(item)
        return packages
    except Exception as e:
        return []

def get_nodes(package):
    """获取指定包中的所有可执行文件"""
    try:
        cmd = ['ros2', 'pkg', 'executables', package]
        result = subprocess.run(cmd, 
                              capture_output=True, text=True)
        if result.returncode == 0:
            nodes = []
            for line in result.stdout.splitlines():
                if line.strip():
                    node = line[len(package)+1:] if line.startswith(package) else line
                    nodes.append(node)
            return nodes
        return []
    except Exception as e:
        return []

def main():
    print("当前工作空间包和节点列表")
    print("=" * 50)
    
    packages = get_workspace_packages()
    if not packages:
        print("未找到任何包，请确保在正确的工作空间目录下运行")
        return
        
    for package in packages:
        print(f"\n包名: {package}")
        nodes = get_nodes(package)
        if nodes:
            print("节点:")
            for node in nodes:
                print(f"  - {node}")
        else:
            print("  (无节点)")

if __name__ == "__main__":
    main() 