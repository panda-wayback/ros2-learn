#!/usr/bin/env python3
import os
import sys
import shutil
import re

def is_valid_package_name(name):
    """检查包名是否符合ROS2命名规范"""
    return bool(re.match(r'^[a-z][a-z0-9_]*$', name))

def replace_in_file(file_path, old_name, new_name):
    """替换文件中的包名"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    content = content.replace(old_name, new_name)
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)

def create_package_from_template(new_package_name):
    """从模板创建新的ROS2包"""
    # 获取当前工作目录（应该是工作空间的根目录）
    workspace_dir = os.getcwd()
    src_dir = os.path.join(workspace_dir, 'src')
    template_dir = os.path.join(src_dir, 'ros2_template')
    new_package_dir = os.path.join(src_dir, new_package_name)
    
    # 检查模板目录是否存在
    if not os.path.exists(template_dir):
        print(f"错误：找不到模板目录 {template_dir}")
        return False
    
    # 检查新包名是否已存在
    if os.path.exists(new_package_dir):
        print(f"错误：包 {new_package_name} 已经存在")
        return False
    
    try:
        # 复制模板目录
        shutil.copytree(template_dir, new_package_dir)
        
        # 重命名包目录
        old_package_dir = os.path.join(new_package_dir, 'ros2_template')
        if os.path.exists(old_package_dir):
            new_module_dir = os.path.join(new_package_dir, new_package_name)
            os.rename(old_package_dir, new_module_dir)
        
        # 更新文件内容
        files_to_update = [
            os.path.join(new_package_dir, 'setup.py'),
            os.path.join(new_package_dir, 'package.xml'),
            os.path.join(new_package_dir, 'setup.cfg'),
        ]
        
        for file_path in files_to_update:
            if os.path.exists(file_path):
                replace_in_file(file_path, 'ros2_template', new_package_name)
        
        # 更新resource目录
        resource_file = os.path.join(new_package_dir, 'resource', 'ros2_template')
        if os.path.exists(resource_file):
            new_resource_file = os.path.join(new_package_dir, 'resource', new_package_name)
            os.rename(resource_file, new_resource_file)
        
        print(f"成功创建新包：{new_package_name}")
        print("\n后续步骤：")
        print(f"1. cd {workspace_dir}")
        print("2. colcon build")
        print("3. source install/setup.bash")
        print(f"4. ros2 run {new_package_name} template_node")
        return True
        
    except Exception as e:
        print(f"错误：创建包时出错 - {str(e)}")
        # 如果出错，尝试清理已创建的目录
        if os.path.exists(new_package_dir):
            shutil.rmtree(new_package_dir)
        return False

def main():
    if len(sys.argv) != 2:
        print("用法: python3 create_package.py <新包名>")
        print("示例: python3 create_package.py my_robot_controller")
        sys.exit(1)
    
    new_package_name = sys.argv[1]
    
    if not is_valid_package_name(new_package_name):
        print("错误：包名无效。包名必须：")
        print("- 只包含小写字母、数字和下划线")
        print("- 以字母开头")
        sys.exit(1)
    
    create_package_from_template(new_package_name)

if __name__ == '__main__':
    main() 