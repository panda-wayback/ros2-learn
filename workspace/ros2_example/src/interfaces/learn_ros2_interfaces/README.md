# learn_ros2_interfaces - 自定义消息类型学习包

## 📖 概述

这个包包含了用于学习ROS 2自定义消息类型的**最简单示例**，包括：
- **消息（Message）**：用于发布-订阅通信
- **服务（Service）**：用于请求-响应通信  
- **动作（Action）**：用于长时间任务通信

## 📁 消息类型

### SimpleMessage.msg - 简单消息示例
```msg
# 简单消息示例
# 用于学习基本数据类型

# 字符串
string name

# 整数
int32 number

# 浮点数
float32 value

# 布尔值
bool flag
```

## 🔧 服务类型

### SimpleService.srv - 简单服务示例
**请求：**
```msg
int32 a
int32 b
```

**响应：**
```msg
int32 result
```

## 🎯 动作类型

### SimpleAction.action - 简单动作示例
**目标：**
```msg
int32 target_number
```

**结果：**
```msg
int32 final_number
bool success
```

**反馈：**
```msg
int32 current_number
```

## 🚀 构建和使用

### 1. 构建接口包
```bash
cd /root/ros2-learn/workspace/ros2_example
colcon build --packages-select learn_ros2_interfaces
source install/setup.bash
```

### 2. 查看接口定义
```bash
# 查看消息定义
ros2 interface show learn_ros2_interfaces/msg/SimpleMessage

# 查看服务定义
ros2 interface show learn_ros2_interfaces/srv/SimpleService

# 查看动作定义
ros2 interface show learn_ros2_interfaces/action/SimpleAction
```

### 3. 列出所有接口
```bash
ros2 interface list | grep learn_ros2_interfaces
```

## 💡 学习要点

### 基本数据类型
- `bool`：布尔值（true/false）
- `int32`：32位整数
- `float32`：32位浮点数
- `string`：字符串

### 接口结构
- **消息（.msg）**：只有数据定义
- **服务（.srv）**：请求 + `---` + 响应
- **动作（.action）**：目标 + `---` + 结果 + `---` + 反馈

### 命名规范
- 文件名使用PascalCase（如：SimpleMessage.msg）
- 字段名使用snake_case（如：target_number）

## 🔗 在其他包中使用

在其他ROS 2包中使用这些接口：

### 1. 在package.xml中添加依赖
```xml
<depend>learn_ros2_interfaces</depend>
```

### 2. 在Python代码中导入
```python
from learn_ros2_interfaces.msg import SimpleMessage
from learn_ros2_interfaces.srv import SimpleService
from learn_ros2_interfaces.action import SimpleAction
```

### 3. 在CMakeLists.txt中添加依赖
```cmake
find_package(learn_ros2_interfaces REQUIRED)
``` 