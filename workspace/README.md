# ROS2 Turtlesim 环境配置指南

## 环境要求
- Ubuntu 系统
- VNC 服务器
- ROS2 Iron 版本

## 快速配置步骤

### 1. 安装 VNC 服务器
```bash
# 安装 VNC 服务器
sudo apt update
sudo apt install -y tightvncserver

# 启动 VNC 服务器（设置密码）
vncserver :1
```

### 2. 安装 ROS2 Iron
```bash
# 设置语言环境
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 仓库
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2
sudo apt update
sudo apt install ros-iron-desktop
```

### 3. 配置环境变量
```bash
# 编辑 .bashrc
nano ~/.bashrc

# 添加以下内容到文件末尾
export DISPLAY=:1
source /opt/ros/iron/setup.bash

# 使配置生效
source ~/.bashrc
```

### 4. 运行 Turtlesim
```bash
# 启动 turtlesim
ros2 run turtlesim turtlesim_node

# 在另一个终端中运行乌龟控制节点
ros2 run turtlesim turtle_teleop_key
```

## 常见问题解决

### 1. VNC 连接问题
- 确保 VNC 服务器正在运行：`vncserver -list`
- 检查防火墙设置
- 确保使用正确的端口号（通常是 5901）

### 2. 显示问题
如果看不到 turtlesim 窗口：
```bash
# 检查 DISPLAY 设置
echo $DISPLAY

# 如果显示为空，设置 DISPLAY
export DISPLAY=:1
```

### 3. ROS2 命令未找到
如果提示找不到 ros2 命令：
```bash
source /opt/ros/iron/setup.bash
```

## 参考链接
- [ROS2 官方文档](https://docs.ros.org/en/iron/index.html)
- [Turtlesim 教程](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) 