#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 Python 包的安装配置文件
此文件定义了包的元数据、依赖关系和可执行入口点
"""

from setuptools import setup, find_packages
import os

# 自动获取当前目录名作为包名
# __file__ 是当前文件的路径
# os.path.abspath() 获取绝对路径
# os.path.dirname() 获取目录路径
# os.path.basename() 获取目录名
package_name = os.path.basename(os.path.dirname(os.path.abspath(__file__)))

setup(
    # 包的基本信息
    name=package_name,  # 包名，与目录名相同
    version='0.0.0',    # 版本号，遵循语义化版本规范
    packages=find_packages(),
    python_requires='>=3.6',
    setup_requires=['setuptools>=42'],

    # 数据文件配置
    data_files=[
        # 将包标记文件安装到 ROS2 包索引目录
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 将 package.xml 安装到包的共享目录
        ('share/' + package_name, ['package.xml']),
    ],

    # 依赖配置
    install_requires=['setuptools'],  # 安装时需要的依赖包
    zip_safe=True,                    # 是否支持 zip 安装

    # 包的元数据
    maintainer='root',                # 维护者
    maintainer_email='you@example.com', # 维护者邮箱
    description='TODO: Package description',  # 包描述
    license='TODO: License declaration',      # 许可证声明

    # 测试配置
    tests_require=['pytest'],         # 测试依赖

    # 可执行入口点配置
    # 这些命令可以在安装后直接使用
    entry_points={
        'console_scripts': [
            # 格式: '命令名 = 模块路径:函数名'
            'publisher = nodes.publisher_node.main:main',
            'subscriber = nodes.subscriber_node.main:main',
            'service = nodes.service_node.main:main',
            'utils_example = nodes.utils_node.main:main',
        ],
    },
)
