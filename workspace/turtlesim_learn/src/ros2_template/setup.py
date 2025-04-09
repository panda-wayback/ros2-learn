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
