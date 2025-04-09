#!/bin/bash

# 检查参数
if [ $# -ne 1 ]; then
    echo "Usage: $0 <new_package_name>"
    exit 1
fi

NEW_PACKAGE=$1
TEMPLATE_DIR="src/ros2_template"
NEW_PACKAGE_DIR="src/$NEW_PACKAGE"

# 检查包名是否合法
if [[ ! $NEW_PACKAGE =~ ^[a-z][a-z0-9_]*$ ]]; then
    echo "Error: Package name must be lowercase with underscores"
    exit 1
fi

# 复制模板
echo "Creating new package: $NEW_PACKAGE"
cp -r $TEMPLATE_DIR $NEW_PACKAGE_DIR

# 重命名目录
mv $NEW_PACKAGE_DIR/ros2_template $NEW_PACKAGE_DIR/$NEW_PACKAGE

# 重命名节点文件
mv $NEW_PACKAGE_DIR/$NEW_PACKAGE/template_node.py $NEW_PACKAGE_DIR/$NEW_PACKAGE/${NEW_PACKAGE}_node.py

# 创建 resource 目录和文件
mkdir -p $NEW_PACKAGE_DIR/resource
touch $NEW_PACKAGE_DIR/resource/$NEW_PACKAGE

# 创建空的 __init__.py
touch $NEW_PACKAGE_DIR/$NEW_PACKAGE/__init__.py

# 更新 package.xml
cat > $NEW_PACKAGE_DIR/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>$NEW_PACKAGE</name>
  <version>0.0.0</version>
  <description>ROS2 Package: $NEW_PACKAGE</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# 更新 setup.py
cat > $NEW_PACKAGE_DIR/setup.py << EOF
from setuptools import setup
import os
import glob

# 自动获取当前文件夹名作为包名
package_name = os.path.basename(os.path.dirname(os.path.abspath(__file__)))

# 自动扫描包目录下的所有 Python 文件
def get_entry_points():
    entry_points = []
    package_dir = os.path.join(os.path.dirname(__file__), package_name)
    
    # 扫描包目录下的所有 Python 文件
    for py_file in glob.glob(os.path.join(package_dir, "*.py")):
        if not py_file.endswith('__init__.py'):
            # 获取文件名（不含扩展名）
            node_name = os.path.splitext(os.path.basename(py_file))[0]
            # 生成入口点
            entry_point = f'{node_name} = {package_name}.{node_name}:main'
            entry_points.append(entry_point)
    
    return entry_points

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description=f'ROS2 Package: {package_name}',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': get_entry_points(),
    },
)
EOF

# 更新节点文件中的类名（保持下划线）
sed -i "s/TemplateNode/${NEW_PACKAGE}Node/g" $NEW_PACKAGE_DIR/$NEW_PACKAGE/${NEW_PACKAGE}_node.py
sed -i "s/template_node/${NEW_PACKAGE}_node/g" $NEW_PACKAGE_DIR/$NEW_PACKAGE/${NEW_PACKAGE}_node.py

# 创建示例消息和服务目录
mkdir -p $NEW_PACKAGE_DIR/$NEW_PACKAGE/msg
mkdir -p $NEW_PACKAGE_DIR/$NEW_PACKAGE/srv
mkdir -p $NEW_PACKAGE_DIR/$NEW_PACKAGE/action

# 创建示例消息文件
cat > $NEW_PACKAGE_DIR/$NEW_PACKAGE/msg/Example.msg << EOF
string message
int32 number
EOF

# 创建示例服务文件
cat > $NEW_PACKAGE_DIR/$NEW_PACKAGE/srv/Example.srv << EOF
string request
---
string response
EOF

# 创建示例动作文件
cat > $NEW_PACKAGE_DIR/$NEW_PACKAGE/action/Example.action << EOF
# Goal
string goal
---
# Result
string result
---
# Feedback
string feedback
EOF

echo "Package $NEW_PACKAGE created successfully!"
echo "Directory structure:"
echo "  - $NEW_PACKAGE_DIR/$NEW_PACKAGE/msg/Example.msg"
echo "  - $NEW_PACKAGE_DIR/$NEW_PACKAGE/srv/Example.srv"
echo "  - $NEW_PACKAGE_DIR/$NEW_PACKAGE/action/Example.action"
echo "  - $NEW_PACKAGE_DIR/$NEW_PACKAGE/${NEW_PACKAGE}_node.py"
echo "  - $NEW_PACKAGE_DIR/$NEW_PACKAGE/__init__.py"
echo ""
echo "You can now build it with: colcon build --packages-select $NEW_PACKAGE" 