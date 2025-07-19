# ROS 2 Action（动作）示例

## 🤔 Action到底是什么？

### 🍕 用生活中的例子理解Action

想象你去**披萨店点餐**：

1. **你（客户端）**：走进店里说"我要一个披萨"
2. **服务员（服务端）**：接受订单，开始制作
3. **制作过程**：服务员会告诉你"面团正在揉"、"正在放料"、"正在烤制"
4. **最终结果**：披萨做好后给你

这就是Action的完整过程！

### 🔄 为什么需要Action？

**问题场景**：你想让机器人从A点移动到B点

#### ❌ 用Service的问题：
```python
# 机器人，移动到B点！
result = robot_move_service.call("B点")
# 机器人：好的，我到了！
```
**问题**：你不知道机器人是否真的在移动，还是卡住了

#### ✅ 用Action的解决方案：
```python
# 机器人，移动到B点！
goal = robot_move_action.send_goal("B点")

# 机器人实时反馈：
# "正在规划路径..."
# "正在避开障碍物..."
# "距离目标还有5米..."
# "距离目标还有3米..."
# "到达目标！"

result = goal.get_result()
```

### 🎯 Action的核心价值

| 场景 | 用Topic | 用Service | 用Action |
|------|---------|-----------|----------|
| 机器人导航 | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ |
| 机械臂抓取 | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ |
| 图像处理 | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ |
| 文件下载 | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ |

## 📖 什么是Action（动作）？

Action是ROS 2中用于**长时间任务**的通信机制，具有以下特点：

### 🔄 通信模式
- **目标-反馈-结果模式**：客户端发送目标，服务端执行并返回结果
- **异步通信**：客户端发送目标后可以继续其他工作
- **反馈机制**：服务端可以定期发送进度反馈
- **可取消**：在任务执行过程中可以取消任务


### 🆚 与其他通信机制的区别

| 特性 | 话题（Topic） | 服务（Service） | 动作（Action） |
|------|---------------|-----------------|----------------|
| 通信模式 | 发布-订阅 | 请求-响应 | 目标-反馈-结果 |
| 同步性 | 异步 | 异步 | 异步 |
| 反馈 | 无 | 无 | 有 |
| 可取消 | 否 | 否 | 是 |
| 适用场景 | 持续数据流 | 一次性任务 | 长时间任务 |

## 🚀 运行示例

### 1. 构建项目
```bash
cd /root/ros2-learn/workspace/ros2_example
colcon build --packages-select learn_ros2
source install/setup.bash
```

### 2. 启动Action服务端
```bash
# 终端1：启动Action服务端
ros2 run learn_ros2 simple_action_server
```

### 3. 运行Action客户端
```bash
# 终端2：运行Action客户端
ros2 run learn_ros2 simple_action_client
```

### 4. 使用命令行工具测试
```bash
# 终端3：查看Action列表
ros2 action list

# 查看Action信息
ros2 action info /simple_count

# 发送Action目标
ros2 action send_goal /simple_count example_interfaces/action/Fibonacci "{order: 5}"

# 发送Action目标并监听反馈
ros2 action send_goal /simple_count example_interfaces/action/Fibonacci "{order: 5}" --feedback
```

## 📁 文件说明

### 核心文件
- `simple_action_server.py` - Action服务端节点，执行计数任务
- `simple_action_client.py` - Action客户端节点，发送目标并接收反馈

### Action类型
本示例使用ROS 2自带的 `example_interfaces.action.Fibonacci`：
- **目标**：`order` - 要计数到的数字
- **结果**：`sequence` - 完整的计数序列 [0,1,2,3,4,5]
- **反馈**：`sequence` - 当前已计数的序列

## 🔧 常用命令

### 查看Action列表
```bash
ros2 action list
```

### 查看Action信息
```bash
ros2 action info /simple_count
```

### 发送Action目标
```bash
ros2 action send_goal /simple_count example_interfaces/action/Fibonacci "{order: 10}"
```

### 发送Action目标并监听反馈
```bash
ros2 action send_goal /simple_count example_interfaces/action/Fibonacci "{order: 10}" --feedback
```

## 💡 学习要点

### 1. Action服务端创建
```python
# 创建Action服务端
self._action_server = ActionServer(
    self,
    Fibonacci,              # Action类型（ROS 2自带）
    'simple_count',         # Action名称
    self.execute_callback   # 执行回调函数
)
```

### 2. Action客户端创建
```python
# 创建Action客户端
self._action_client = ActionClient(self, Fibonacci, 'simple_count')

# 等待服务端可用
self._action_client.wait_for_server()
```

### 3. 发送目标
```python
# 创建目标消息
goal_msg = Fibonacci.Goal()
goal_msg.order = 5

# 发送目标
send_goal_future = self._action_client.send_goal_async(
    goal_msg,
    feedback_callback=self.feedback_callback
)
```

### 4. 处理反馈
```python
def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info(f'当前计数: {feedback.sequence}')
```

### 5. 获取结果
```python
# 等待结果
get_result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(self, get_result_future)
result = get_result_future.result().result
```

### 6. 取消任务
```python
# 取消目标
cancel_future = goal_handle.cancel_goal_async()
rclpy.spin_until_future_complete(self, cancel_future)
```

## 🎯 实际应用场景

1. **机器人导航**：发送目标位置，接收路径规划反馈
2. **机械臂抓取**：发送抓取目标，接收运动反馈
3. **图像处理**：发送处理任务，接收进度反馈
4. **数据计算**：发送计算任务，接收计算进度
5. **文件传输**：发送传输任务，接收传输进度

## 🔍 调试技巧

1. **使用 `ros2 action list`** 查看所有可用Action
2. **使用 `ros2 action info`** 查看Action详细信息
3. **使用 `ros2 action send_goal`** 手动测试Action
4. **查看节点日志** 了解Action执行情况
5. **使用 `--feedback` 参数** 实时查看反馈

## 🌟 简单示例说明

这个简单的Action示例演示了：

1. **目标**：客户端发送一个数字（比如5）
2. **反馈**：服务端每0.5秒发送一次当前计数进度
3. **结果**：服务端完成计数后返回完整序列 [0,1,2,3,4,5]

整个过程展示了Action的完整生命周期：**目标 → 反馈 → 结果** 