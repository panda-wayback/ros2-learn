FROM althack/ros2:humble-gazebo

# 安装 VNC 和必要的包
RUN apt-get update && apt-get install -y \
    gz-garden \
    tigervnc-standalone-server \
    tigervnc-common \
    xfce4 \
    xfce4-terminal \
    dbus-x11 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 设置 VNC 密码（默认为 'vncpassword'）
RUN mkdir ~/.vnc && echo "vncpassword" | vncpasswd -f > ~/.vnc/passwd && chmod 600 ~/.vnc/passwd

# 设置环境变量
ENV DISPLAY=:1
ENV USER=root

# 设置工作目录
WORKDIR /workspace

# 复制 requirements.txt 并安装 Python 依赖
COPY workspace/ros2_ws/requirements.txt /workspace/requirements.txt
RUN pip3 install -r /workspace/requirements.txt

# 源设置文件
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 创建启动脚本
RUN echo '#!/bin/bash\nvncserver :1 -geometry 1920x1080 -depth 24 -localhost no\ntail -f /dev/null' > /start.sh \
    && chmod +x /start.sh

CMD ["/start.sh"] 