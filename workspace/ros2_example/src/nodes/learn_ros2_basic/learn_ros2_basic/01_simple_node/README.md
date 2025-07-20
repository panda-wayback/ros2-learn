# 01_simple_node - 最简单的ROS2节点

## 概述

这是你的第一个ROS2节点示例，展示了ROS2节点的基本结构和运行方式。

## 学习目标

通过这个节点，你将学习到：

1. **ROS2节点的基本概念**
   - 什么是节点（Node）
   - 节点的生命周期
   - 节点的基本结构

2. **ROS2编程基础**
   - 如何导入ROS2库
   - 如何创建节点类
   - 如何初始化和运行节点

3. **ROS2核心机制**
   - 定时器（Timer）的使用
   - 日志（Logger）系统
   - 异常处理

## 节点功能

这个节点实现了以下功能：

- ✅ 节点初始化和启动
- ✅ 每秒输出心跳信息
- ✅ 显示当前时间戳
- ✅ 优雅的关闭处理

## 运行方法

### 方法1：使用colcon构建和运行

```bash
# 1. 构建包
colcon build --packages-select learn_ros2_basic

# 2. 设置环境
source install/setup.bash

# 3. 运行节点
ros2 run learn_ros2_basic 01_simple_node
```

### 方法2：直接运行Python文件

```bash
# 1. 设置ROS2环境
source /opt/ros/humble/setup.bash

# 2. 直接运行Python文件
python3 src/nodes/learn_ros2_basic/learn_ros2_basic/01_simple_node/main.py
```

## 预期输出

运行节点后，你应该看到类似以下的输出：

```
[INFO] [simple_node]: 01_simple_node 已启动！
[INFO] [simple_node]: 这是一个最简单的ROS2节点示例
[INFO] [simple_node]: 定时器已创建，每秒执行一次回调
[INFO] [simple_node]: 开始运行节点...
[INFO] [simple_node]: 节点正在运行... 时间戳: 1703123456.789
[INFO] [simple_node]: 节点正在运行... 时间戳: 1703123457.789
[INFO] [simple_node]: 节点正在运行... 时间戳: 1703123458.789
...
```

## 停止节点

使用 `Ctrl+C` 来停止节点运行。

## 代码结构

```
01_simple_node/
├── __init__.py          # 包初始化文件
├── main.py             # 主程序文件
└── README.md           # 说明文档
```

## 关键概念解释

### 1. 节点（Node）
- ROS2中的基本计算单元
- 每个节点都是一个独立的进程
- 节点之间通过ROS2的通信机制进行交互

### 2. 定时器（Timer）
- 用于周期性执行任务
- 可以设置执行频率
- 是ROS2中常用的机制

### 3. 日志（Logger）
- 用于输出调试和状态信息
- 支持不同的日志级别
- 便于调试和监控

## 下一步学习

完成这个节点后，你可以继续学习：

1. **02_topic_publisher** - 学习话题发布
2. **03_topic_subscriber** - 学习话题订阅
3. **04_service_server** - 学习服务服务器
4. **05_action_server** - 学习动作服务器

## 注意事项

- 确保ROS2环境已正确安装
- 节点名称在系统中必须唯一
- 使用 `Ctrl+C` 优雅地停止节点 