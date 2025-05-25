# ROS2 Launch 文件使用说明

本目录包含了 example_pkg 包的所有 launch 文件，用于启动和管理 ROS2 节点。

## 目录结构

```
launch/
├── README.md
├── all_nodes.launch.py          # 主 launch 文件，用于启动所有节点
└── nodes/                       # 按功能分组的节点 launch 文件
    ├── publisher_subscriber.launch.py  # 发布者和订阅者节点
    └── service.launch.py        # 服务和服务客户端节点
```

## 使用方法

### 1. 启动所有节点

```bash
ros2 launch example_pkg all_nodes.launch.py
```

### 2. 选择性启动节点

可以通过参数控制启动哪些节点：

```bash
# 只启动发布者和订阅者
ros2 launch example_pkg all_nodes.launch.py use_service:=false

# 只启动服务节点
ros2 launch example_pkg all_nodes.launch.py use_pub_sub:=false
```

### 3. 单独启动功能模块

也可以直接启动单个功能模块：

```bash
# 启动发布者和订阅者
ros2 launch example_pkg nodes/publisher_subscriber.launch.py

# 启动服务节点
ros2 launch example_pkg nodes/service.launch.py
```

## 节点说明

### 发布者/订阅者节点 (publisher_subscriber.launch.py)

- **publisher_node**
  - 功能：发布消息
  - 参数：
    - `publish_frequency`: 发布频率（默认：1.0 Hz）
    - `message_prefix`: 消息前缀（默认：'Hello from publisher'）

- **subscriber_node**
  - 功能：订阅并处理消息
  - 输出：在屏幕上显示接收到的消息

### 服务节点 (service.launch.py)

- **service_node**
  - 功能：提供服务
  - 输出：在屏幕上显示服务请求

- **service_client_node**
  - 功能：调用服务
  - 参数：
    - `request_interval`: 请求间隔（默认：2.0 秒）

## 参数配置

### 主 launch 文件参数 (all_nodes.launch.py)

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| use_pub_sub | bool | true | 是否启动发布者和订阅者节点 |
| use_service | bool | true | 是否启动服务节点 |

### 如何修改参数

1. 通过命令行：
```bash
ros2 launch example_pkg all_nodes.launch.py use_pub_sub:=false
```

2. 通过参数文件：
创建 YAML 文件，例如 `config.yaml`：
```yaml
/**:
  ros__parameters:
    publish_frequency: 2.0
    message_prefix: "Custom message"
```

然后在 launch 文件中使用：
```python
Node(
    ...
    parameters=['path/to/config.yaml']
)
```

## 调试技巧

1. 查看节点输出：
```bash
ros2 launch example_pkg all_nodes.launch.py --debug
```

2. 查看节点信息：
```bash
ros2 node list
ros2 node info /node_name
```

3. 查看话题：
```bash
ros2 topic list
ros2 topic echo /topic_name
```

4. 查看服务：
```bash
ros2 service list
ros2 service call /service_name service_type
```

## 注意事项

1. 确保在运行 launch 文件前已经构建并安装了包
2. 确保已经 source 了工作空间的环境
3. 如果修改了 launch 文件，需要重新构建包
4. 使用 `ros2 launch --debug` 可以获取更详细的启动信息

## 常见问题

1. **找不到 launch 文件**
   - 检查包是否正确安装
   - 检查 launch 文件是否在正确的位置
   - 运行 `colcon build` 重新构建包

2. **节点启动失败**
   - 检查节点名称是否正确
   - 检查参数配置是否正确
   - 查看节点输出获取错误信息

3. **参数不生效**
   - 检查参数名称是否正确
   - 检查参数类型是否正确
   - 确保参数文件格式正确 