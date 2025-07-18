#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动扫描 src 目录下的所有包和 build 目录下的接口包，并更新 .vscode/settings.json 的 extraPaths
"""

import os
import json
import glob
from pathlib import Path

def find_ros2_packages(src_dir="src"):
    """扫描 src 目录下的所有 ROS2 包"""
    packages = []
    
    if not os.path.exists(src_dir):
        return packages
    
    # 递归搜索所有包含 setup.py 的目录
    for setup_file in glob.glob(f"{src_dir}/**/setup.py", recursive=True):
        package_dir = os.path.dirname(setup_file)
        # 获取相对于工作区根目录的路径
        relative_path = os.path.relpath(package_dir, ".")
        packages.append(f"./{relative_path}")
    
    return packages

def find_build_interfaces(build_dir="build"):
    """扫描 build 目录下的接口包"""
    interfaces = []
    
    if not os.path.exists(build_dir):
        return interfaces
    
    # 只搜索主要的 rosidl_generator_py 目录，过滤掉 CMakeFiles
    for interface_dir in glob.glob(f"{build_dir}/*/rosidl_generator_py"):
        # 检查路径中是否包含 CMakeFiles，如果包含则跳过
        if "CMakeFiles" not in interface_dir:
            # 获取相对于工作区根目录的路径
            relative_path = os.path.relpath(interface_dir, ".")
            interfaces.append(f"./{relative_path}")
    
    # 搜索其他可能的接口目录（lib 目录）
    for interface_dir in glob.glob(f"{build_dir}/*/build/lib"):
        # 检查是否是 Python 包目录
        if os.path.exists(os.path.join(interface_dir, "__init__.py")) or \
           any(f.endswith('.py') for f in os.listdir(interface_dir) if os.path.isfile(os.path.join(interface_dir, f))):
            relative_path = os.path.relpath(interface_dir, ".")
            interfaces.append(f"./{relative_path}")
    
    return interfaces

def update_vscode_settings():
    """更新 .vscode/settings.json 文件"""
    settings_file = ".vscode/settings.json"
    
    # 读取现有配置
    if os.path.exists(settings_file):
        with open(settings_file, 'r', encoding='utf-8') as f:
            settings = json.load(f)
    else:
        settings = {}
    
    # 查找所有包和接口
    packages = find_ros2_packages()
    interfaces = find_build_interfaces()
    
    # 更新 extraPaths
    extra_paths = ["."]  # 保留根目录
    
    # 添加所有找到的接口
    extra_paths.extend(interfaces)
    
    # 添加所有找到的包
    extra_paths.extend(packages)
    
    # 更新配置
    settings["python.analysis.extraPaths"] = extra_paths
    settings["python.analysis.autoSearchPaths"] = True
    settings["cursorpyright.analysis.extraPaths"] = extra_paths
    settings["cursorpyright.analysis.autoSearchPaths"] = True
    
    # 确保 .vscode 目录存在
    os.makedirs(".vscode", exist_ok=True)
    
    # 写入配置
    with open(settings_file, 'w', encoding='utf-8') as f:
        json.dump(settings, f, indent=4, ensure_ascii=False)
    
    print(f"已更新 {settings_file}")
    print(f"找到的接口包: {interfaces}")
    print(f"找到的源码包: {packages}")
    print(f"总共 {len(extra_paths)} 个路径")

if __name__ == "__main__":
    update_vscode_settings() 