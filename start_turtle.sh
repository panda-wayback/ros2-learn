#!/bin/bash

# 进入工作空间
cd /workspace/ros2_ws

# 编译包
echo "正在编译包..."
colcon build

# 设置环境变量
source install/setup.bash

# 启动乌龟模拟器
echo "启动乌龟模拟器..."
ros2 run turtlesim turtlesim_node &

# 等待模拟器启动
sleep 2

# 启动控制器
echo "启动控制器..."
ros2 run my_first_py_pkg turtle_controller 