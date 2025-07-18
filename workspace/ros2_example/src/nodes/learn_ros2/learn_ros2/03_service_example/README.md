# ROS 2 服务（Service）示例

## 📖 什么是服务（Service）？

服务是ROS 2中的一种**请求-响应**通信机制，具有以下特点：

### 🔄 通信模式
- **请求-响应模式**：客户端发送请求，服务端处理并返回响应
- **同步通信**：客户端会等待服务端处理完成
- **一次性任务**：适合计算、查询、状态检查等任务

### 🆚 与话题（Topic）的区别

| 特性 | 话题（Topic） | 服务（Service） |
|------|---------------|-----------------|
| 通信模式 | 发布-订阅 | 请求-响应 |
| 同步性 | 异步 | 同步 |
| 返回值 | 无 | 有 |
| 适用场景 | 持续数据流 | 一次性任务 |
| 连接方式 | 多对多 | 一对多 |

## 🚀 运行示例

### 1. 构建项目
```bash
cd /root/ros2-learn/workspace/ros2_example
colcon build --packages-select learn_ros2
source install/setup.bash
```

### 2. 启动服务端
```bash
# 终端1：启动服务端
ros2 run learn_ros2 service_server
```

### 3. 测试服务（三种方式）

#### 方式1：使用命令行工具
```bash
# 终端2：使用ros2 service call命令测试
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

#### 方式2：使用自动测试客户端
```bash
# 终端2：运行自动测试客户端
ros2 run learn_ros2 service_client
```

#### 方式3：使用交互式客户端
```bash
# 终端2：运行交互式客户端
ros2 run learn_ros2 interactive_client
```

## 📁 文件说明

### 核心文件
- `service_server.py` - 服务端节点，提供加法计算服务
- `service_client.py` - 自动测试客户端，发送预设的测试数据
- `interactive_client.py` - 交互式客户端，允许用户输入数字

### 服务类型
本示例使用ROS 2内置的 `example_interfaces.srv.AddTwoInts` 服务：
- **请求**：包含两个整数 `a` 和 `b`
- **响应**：包含一个整数 `sum`（a + b的结果）

## 🔧 常用命令

### 查看服务列表
```bash
ros2 service list
```

### 查看服务类型
```bash
ros2 service type /add_two_ints
```

### 查看服务信息
```bash
ros2 service info /add_two_ints
```

### 手动调用服务
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

## 💡 学习要点

### 1. 服务端创建
```python
# 创建服务端
self.srv = self.create_service(
    AddTwoInts,           # 服务类型
    'add_two_ints',       # 服务名称
    self.callback_function # 回调函数
)
```

### 2. 客户端创建
```python
# 创建客户端
self.client = self.create_client(AddTwoInts, 'add_two_ints')

# 等待服务可用
while not self.client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('等待服务端启动...')
```

### 3. 发送请求
```python
# 创建请求
request = AddTwoInts.Request()
request.a = 5
request.b = 3

# 发送请求
future = self.client.call_async(request)
rclpy.spin_until_future_complete(self, future)
response = future.result()
```

### 4. 处理请求（服务端）
```python
def callback_function(self, request, response):
    # 处理请求
    result = request.a + request.b
    
    # 设置响应
    response.sum = result
    
    return response
```

## 🎯 实际应用场景

1. **机器人控制**：发送移动命令，等待执行完成
2. **状态查询**：查询机器人当前状态
3. **参数设置**：设置机器人参数
4. **计算服务**：提供数学计算功能
5. **数据库查询**：查询数据库信息

## 🔍 调试技巧

1. **使用 `ros2 service list`** 查看所有可用服务
2. **使用 `ros2 service call`** 手动测试服务
3. **查看节点日志** 了解服务调用情况
4. **使用 `ros2 node info`** 查看节点详细信息 