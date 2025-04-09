# ROS2 开发模板

这是一个基于 ROS2 的开发模板项目，提供了常用的开发工具和命令。

## 项目结构

```
ros2_example/
├── src/                    # ROS2 包源代码目录
│   ├── basic_communication/ # 基础通信示例包
│   └── example_pkg/        # 示例包
├── scripts/                # 工具脚本目录
│   └── list_nodes.py      # 节点列表工具
├── build/                  # 编译输出目录
├── install/                # 安装目录
├── log/                    # 日志目录
└── Makefile               # 项目构建和工具脚本
```

## 环境要求

- ROS2 (推荐使用 Humble 或更高版本)
- Python 3.8+
- colcon 构建工具

## 快速开始

1. 克隆项目
```bash
git clone <repository-url>
cd ros2_example
```

2. 编译项目
```bash
make build
```

3. 设置环境
```bash
make setup
```

## 常用命令

### 构建相关
- `make build` - 编译所有包
- `make clean` - 清理编译文件
- `make setup` - 设置环境变量

### 运行节点
有两种方式运行节点：

1. 使用完整参数：
```bash
make run PKG=<包名> NODE=<节点名>
```

2. 使用简写方式：
```bash
make run <包名>-<节点名>
```

例如：
```bash
make run basic_communication-publisher
```

### 查看信息
- `make list-nodes` - 查看所有节点
- `make list-topics` - 查看所有主题
- `make list-services` - 查看所有服务
- `make list-params` - 查看所有参数
- `make list-all` - 查看所有包和节点

## 开发指南

### 添加新包
1. 在 `src` 目录下创建新的包
2. 使用 `colcon build` 编译
3. 使用 `make setup` 更新环境

### 包结构
每个 ROS2 包应包含：
- `package.xml` - 包描述文件
- `CMakeLists.txt` - 构建配置
- `src/` - 源代码目录
- `include/` - 头文件目录
- `launch/` - 启动文件目录

## 工具脚本

### list_nodes.py
用于列出所有可用的节点，支持以下功能：
- 显示包名和节点名
- 显示节点类型
- 显示节点描述

## 注意事项

1. 运行节点前确保已执行 `make setup`
2. 修改代码后需要重新编译
3. 使用 `make clean` 清理编译文件时需谨慎

## 常见问题

1. 找不到节点？
   - 确保已执行 `make setup`
   - 检查包名和节点名是否正确
   - 确认节点是否已编译

2. 编译失败？
   - 检查依赖是否安装
   - 查看编译日志
   - 尝试清理后重新编译

## 贡献指南

1. Fork 项目
2. 创建特性分支
3. 提交更改
4. 推送到分支
5. 创建 Pull Request

## 许可证

[添加许可证信息] 