# ROS 2 话题和消息示例

这个示例演示了ROS 2中最基本的通信机制：**话题（Topic）**和**消息（Message）**。

## 📋 概念说明

### 话题（Topic）
- 话题是ROS 2中节点间通信的主要方式
- 采用**发布-订阅（Publish-Subscribe）**模式
- 一个话题可以有多个发布者和多个订阅者
- 话题名称通常以`/`开头，如`/chatter`

### 消息（Message）
- 消息是节点间传递的数据格式
- 有预定义的数据结构
- 常用消息类型：
  - `std_msgs/String`: 字符串消息
  - `std_msgs/Int32`: 整数消息
  - `geometry_msgs/Twist`: 速度消息
  - `sensor_msgs/Image`: 图像消息

## 🚀 运行示例

### 方法1：使用启动文件（推荐）

```bash
# 在workspace根目录下
source install/setup.bash
ros2 launch learn_ros2 topic_demo.launch.py
```

### 方法2：分别启动节点

**终端1 - 启动发布者：**
```bash
source install/setup.bash
ros2 run learn_ros2 publisher_node
```

**终端2 - 启动订阅者：**
```bash
source install/setup.bash
ros2 run learn_ros2 subscriber_node
```

## 🔍 观察和调试

### 查看话题列表
```bash
ros2 topic list
```

### 查看话题信息
```bash
ros2 topic info /chatter
```

### 手动发布消息
```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from command line!'"
```

### 监听话题消息
```bash
ros2 topic echo /chatter
```

### 查看话题频率
```bash
ros2 topic hz /chatter
```

## 📁 文件结构

```
02_topic_message/
├── __init__.py              # 包初始化文件
├── publisher_node.py        # 发布者节点
├── subscriber_node.py       # 订阅者节点
├── topic_demo.launch.py     # 启动文件
└── README.md               # 说明文档
```

## 💡 代码要点

### 发布者节点关键代码
```python
# 创建发布者
self.publisher = self.create_publisher(String, 'chatter', 10)

# 发布消息
self.publisher.publish(msg)
```

### 订阅者节点关键代码
```python
# 创建订阅者
self.subscription = self.create_subscription(
    String, 'chatter', self.listener_callback, 10
)

# 消息回调函数
def listener_callback(self, msg):
    # 处理接收到的消息
    pass
```

## 🎯 学习目标

通过这个示例，你应该能够：

1. ✅ 理解话题和消息的基本概念
2. ✅ 创建发布者节点发布消息
3. ✅ 创建订阅者节点接收消息
4. ✅ 使用启动文件同时启动多个节点
5. ✅ 使用ROS 2命令行工具调试和观察话题

## 🔧 扩展练习

1. **修改消息频率**：将发布频率从1秒改为0.5秒
2. **添加新消息类型**：使用`std_msgs/Int32`发布数字消息
3. **多个订阅者**：启动多个订阅者节点，观察消息分发
4. **自定义处理**：在订阅者中添加消息内容分析逻辑

## 🚨 常见问题

**Q: 为什么看不到消息？**
A: 确保两个节点都已启动，并且话题名称一致。

**Q: 如何停止程序？**
A: 在终端中按 `Ctrl+C` 停止节点。

**Q: 消息丢失怎么办？**
A: 增加队列大小（如从10改为100），或提高处理速度。 