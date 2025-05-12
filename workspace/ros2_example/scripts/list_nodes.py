#!/usr/bin/env python3

import subprocess
import json
import os

def get_workspace_packages():
    """获取src/pkgs目录下的包"""
    try:
        workspace_path = os.getcwd()
        pkgs_path = os.path.join(workspace_path, 'src', 'pkgs')
        packages = []
        for item in os.listdir(pkgs_path):
            item_path = os.path.join(pkgs_path, item)
            if os.path.isdir(item_path) and os.path.exists(os.path.join(item_path, 'package.xml')):
                packages.append(item)
        return packages
    except Exception as e:
        print(f"错误: {str(e)}")
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
        print(f"获取节点时出错: {str(e)}")
        return []

def main():
    print("src/pkgs目录下的包和节点列表")
    print("=" * 50)
    
    packages = get_workspace_packages()
    if not packages:
        print("未在src/pkgs目录下找到任何包")
        return
        
    for package in packages:
        print(f"\n包名: {package}")
        nodes = get_nodes(package)
        if nodes:
            print("节点:")
            for node in nodes:
                print(f"  - {node} --   make run {package}-{node}")
        else:
            print("  (无节点)")

if __name__ == "__main__":
    main() 