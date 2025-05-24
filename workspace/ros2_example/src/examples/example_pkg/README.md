# ROS2 Example Package

## 项目简介
这是一个ROS2示例包，用于展示ROS2 Python包的标准结构和开发规范。

## 目录结构
```
example_pkg/
├── package.xml          # 包配置文件
├── setup.py            # Python包安装配置
├── setup.cfg           # Python包配置
├── README.md           # 项目说明文档
├── resource/           # 资源文件目录
├── test/              # 测试文件目录
├── interface/         # 接口定义目录
│   ├── action/       # Action接口定义
│   │   └── *.action  # Action文件
│   ├── msg/          # 消息接口定义
│   │   └── *.msg     # 消息文件
│   └── srv/          # 服务接口定义
│       └── *.srv     # 服务文件
└── example_pkg/       # 主要代码目录
    ├── __init__.py
    ├── publisher_node/
    ├── subscriber_node/
    ├── service_node/
    └── utils_node/
```

## 开发规范

### 1. 包结构规范
- 每个ROS2包必须包含上述基本目录结构
- 所有Python模块必须放在包名对应的目录下
- 每个功能节点应该放在独立的模块中
- 工具类应该放在utils目录下

### 2. 接口定义规范
#### 2.1 消息接口 (msg)
- 位置：`interface/msg/`
- 命名规则：
  - 使用小写字母和下划线
  - 以`.msg`为后缀
  - 例如：`robot_status.msg`
- 内容规范：
  - 每个字段必须指定类型
  - 必须包含注释说明
  - 数组类型必须指定大小
  - 示例：
    ```
    # 机器人状态消息
    uint8 status          # 状态码
    string description    # 状态描述
    float32[3] position   # 位置坐标
    ```

#### 2.2 服务接口 (srv)
- 位置：`interface/srv/`
- 命名规则：
  - 使用小写字母和下划线
  - 以`.srv`为后缀
  - 例如：`robot_control.srv`
- 内容规范：
  - 必须包含请求和响应部分
  - 使用`---`分隔请求和响应
  - 必须包含注释说明
  - 示例：
    ```
    # 请求部分
    float32 target_x     # 目标X坐标
    float32 target_y     # 目标Y坐标
    ---
    # 响应部分
    bool success         # 执行结果
    string message       # 结果描述
    ```

#### 2.3 Action接口 (action)
- 位置：`interface/action/`
- 命名规则：
  - 使用小写字母和下划线
  - 以`.action`为后缀
  - 例如：`robot_navigation.action`
- 内容规范：
  - 必须包含目标、结果和反馈三部分
  - 使用`---`分隔各部分
  - 必须包含注释说明
  - 示例：
    ```
    # 目标部分
    float32 target_x     # 目标X坐标
    float32 target_y     # 目标Y坐标
    ---
    # 结果部分
    bool success         # 执行结果
    string message       # 结果描述
    ---
    # 反馈部分
    float32 current_x    # 当前X坐标
    float32 current_y    # 当前Y坐标
    float32 progress     # 进度百分比
    ```

### 3. 命名规范
- 包名：使用小写字母和下划线
- 节点文件名：使用小写字母和下划线
- 类名：使用驼峰命名法
- 函数和变量：使用小写字母和下划线

### 4. 代码风格
- 遵循PEP 8规范
- 使用4个空格缩进
- 行长度不超过79个字符
- 使用UTF-8编码
- 每个Python文件必须包含文件头注释
- 每个函数必须包含文档字符串
- 复杂逻辑必须包含注释说明

### 5. 依赖管理
- 所有ROS2依赖在package.xml中声明
- Python包依赖在setup.py中声明
- 避免使用通配符版本号

### 6. 测试规范
- 每个功能模块必须有对应的测试文件
- 测试文件放在test目录下
- 测试文件名必须以test_开头

### 7. 版本控制
- 版本号遵循语义化版本规范
- 每次发布新版本必须更新package.xml和setup.py中的版本号

## 安装说明

### 环境要求
- ROS2 Humble
- Python 3.6+
- 依赖包：
  - rclpy
  - std_msgs
  - ros2_common_utils

### 安装步骤
1. 克隆代码到ROS2工作空间
```bash
cd ~/ros2_ws/src
git clone [repository_url]
```

2. 安装依赖
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. 编译
```bash
colcon build --packages-select example_pkg
```

4. 设置环境变量
```bash
source install/setup.bash
```


## 开发指南

### 添加新接口
1. 根据接口类型选择正确的目录（msg/srv/action）
2. 创建新的接口文件，遵循命名规范
3. 在package.xml中添加必要的依赖：
   - 消息接口：`rosidl_default_generators`
   - 服务接口：`rosidl_default_generators`
   - Action接口：`rosidl_default_generators`和`rosidl_typesupport_cpp`
4. 在CMakeLists.txt中添加接口文件
5. 编写对应的测试文件

### 添加新节点
1. 在example_pkg目录下创建新的节点目录
2. 在setup.py中添加新的入口点
3. 在package.xml中添加必要的依赖
4. 编写对应的测试文件
