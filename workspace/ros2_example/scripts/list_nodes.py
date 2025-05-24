#!/usr/bin/env python3

import subprocess
import json
import os
import yaml

def load_config():
    """加载配置文件"""
    config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    return {
        'workspace_paths': [
            {'name': 'examples', 'path': 'src/examples'},
            # {'name': 'pkgs', 'path': 'src/pkgs'}
        ]
    }

def get_workspace_packages():
    """获取所有配置的工作空间路径下的包"""
    try:
        config = load_config()
        workspace_path = os.getcwd()
        all_packages = []
        
        for ws_config in config['workspace_paths']:
            ws_name = ws_config['name']
            ws_path = os.path.join(workspace_path, ws_config['path'])
            
            if not os.path.exists(ws_path):
                print(f"警告: 路径 {ws_path} 不存在")
                continue
                
            packages = []
            for item in os.listdir(ws_path):
                item_path = os.path.join(ws_path, item)
                if os.path.isdir(item_path) and os.path.exists(os.path.join(item_path, 'package.xml')):
                    packages.append({
                        'name': item,
                        'workspace': ws_name,
                        'path': os.path.join(ws_config['path'], item)
                    })
            all_packages.extend(packages)
            
        return all_packages
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
    print("工作空间中的包和节点列表")
    print("=" * 50)
    
    packages = get_workspace_packages()
    if not packages:
        print("未找到任何包")
        return
        
    # 按工作空间分组显示
    workspaces = {}
    for package in packages:
        ws_name = package['workspace']
        if ws_name not in workspaces:
            workspaces[ws_name] = []
        workspaces[ws_name].append(package)
    
    for ws_name, ws_packages in workspaces.items():
        print(f"\n工作空间: {ws_name}")
        print("-" * 30)
        
        for package in ws_packages:
            print(f"\n包名: {package['name']}")
            print(f"路径: {package['path']}")
            nodes = get_nodes(package['name'])
            if nodes:
                print("节点:")
                for node in nodes:
                    print(f"  - {node} --   make run {package['name']}-{node}")
            else:
                print("  (无节点)")

if __name__ == "__main__":
    main() 