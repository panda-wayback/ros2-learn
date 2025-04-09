# ✅ 远程服务器开发环境配置说明（支持 ROS2 + VNC + VSCode）

本说明记录了使用 Ubuntu 服务器成功搭建的图形化远程开发环境，支持：

- 使用 **本地 VSCode** 编辑远程代码
- 使用 **TigerVNC + XFCE** 图形界面运行 GUI 程序（如 RViz、Gazebo）
- 支持开机自动启动 VNC
- 精简美化 root 用户终端

---

## 🧱 环境基础

- 系统：Ubuntu 22.04+
- 用户：`root`
- 用途：远程开发 ROS2 项目
- VNC 分辨率：2560x1440（2K）

---

## 📦 安装与配置步骤

### 1. 安装 XFCE 图形桌面 + VNC

```bash
apt update
apt install xfce4 xfce4-goodies tigervnc-standalone-server -y
```

### 2. 设置 VNC 启动脚本（路径：`/root/.vnc/xstartup`）

```bash
cat << 'EOF' > /root/.vnc/xstartup
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec startxfce4
EOF

chmod +x /root/.vnc/xstartup
```

---

## 🖥️ 启动 VNC（手动方式）

```bash
tigervncserver -kill :1
tigervncserver :1 -localhost no -geometry 2560x1440 -xstartup /root/.vnc/xstartup
```

---

## ⚙️ 设置 VNC 开机自启（Systemd）

### 启动脚本 `/usr/local/bin/vnc-start`

```bash
#!/bin/bash
tigervncserver :1 -localhost no -geometry 2560x1440 -xstartup /root/.vnc/xstartup
```

### 停止脚本 `/usr/local/bin/vnc-stop`

```bash
#!/bin/bash
tigervncserver -kill :1
```

### Systemd 服务 `/etc/systemd/system/vncserver.service`

```ini
[Unit]
Description=VNC Server for root user
After=network.target

[Service]
Type=forking
ExecStart=/usr/local/bin/vnc-start
ExecStop=/usr/local/bin/vnc-stop
User=root
PIDFile=/root/.vnc/%H:1.pid
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

### 启用服务

```bash
systemctl daemon-reexec
systemctl daemon-reload
systemctl enable vncserver
systemctl start vncserver
```

---

## 💻 VSCode 使用 Remote SSH 连接远程服务器

### 安装插件

- Remote - SSH（ID：`ms-vscode-remote.remote-ssh`）

### 配置 `~/.ssh/config`（本地）

```ssh
Host my-server
  HostName 14.103.153.198
  User root
```

---

## 👁‍🗨 图形程序在 VNC 桌面显示

```bash
export DISPLAY=:1
ros2 run rviz2 rviz2
```

---

## 🎨 美化 root 用户终端（Bash）

### 安装补全功能

```bash
apt install bash-completion -y
```

### `.bashrc` 精简美化配置

```bash
# Enable bash completion
if [ -f /etc/bash_completion ]; then
  . /etc/bash_completion
fi

# Git branch in prompt
parse_git_branch() {
  git branch 2>/dev/null | grep \* | sed 's/* \(.*\)/ (\1)/'
}
PS1='\[\e[1;36m\]➜\[\e[0m\] \[\e[1;34m\]\W\[\e[33m\]$(parse_git_branch)\[\e[0m\] \$ '
```

```bash
source ~/.bashrc
```

---

## 🏁 小结

你现在拥有：

- ✅ VSCode 本地开发 + VNC 远程图形界面
- ✅ 自动补全 + 彩色终端 + Git 提示
- ✅ 支持 ROS2 的完整远程开发环境


### 3. 配置环境变量
```bash
# 编辑 .bashrc
nano ~/.bashrc

# 添加以下内容到文件末尾
export DISPLAY=:1

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
