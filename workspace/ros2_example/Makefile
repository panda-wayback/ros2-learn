.PHONY: build clean setup run list-nodes list-topics help list-all

# 默认目标
all: build

# 编译包
build:
	colcon build

# 清理编译文件
clean:
	rm -rf build install log

# 设置环境变量
setup:
	bash -c "source install/setup.bash"

# 运行节点
# 用法: make run PKG=<包名> NODE=<节点名>
run: setup
	bash -c "ros2 run $(PKG) $(NODE)"

# 查看节点列表
list-nodes: setup
	bash -c "ros2 node list"

# 查看主题列表
list-topics: setup
	bash -c "ros2 topic list"

# 查看服务列表
list-services: setup
	bash -c "ros2 service list"

# 查看参数列表
list-params: setup
	bash -c "ros2 param list"

# 列出所有包和节点
list-all: setup
	@echo "正在列出包和节点..."
	@echo "当前工作空间: $(shell pwd)"
	@echo "检查脚本权限..."
	@ls -l scripts/list_nodes.py
	@echo "运行脚本..."
	bash -c "source install/setup.bash && python3 scripts/list_nodes.py"

# 帮助信息
help:
	@echo "ROS2 开发模板 Makefile"
	@echo ""
	@echo "基本命令:"
	@echo "  make build        - 编译所有包"
	@echo "  make clean        - 清理编译文件"
	@echo "  make setup        - 设置环境变量"
	@echo ""
	@echo "运行命令:"
	@echo "  make run PKG=<包名> NODE=<节点名>  - 运行指定节点"
	@echo "  例如: make run PKG=basic_communication NODE=publisher"
	@echo ""
	@echo "查看命令:"
	@echo "  make list-nodes    - 查看所有节点"
	@echo "  make list-topics   - 查看所有主题"
	@echo "  make list-services - 查看所有服务"
	@echo "  make list-params   - 查看所有参数"
	@echo "  make list-all      - 查看所有包和节点" 