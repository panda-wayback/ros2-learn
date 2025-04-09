SHELL := /bin/bash

.PHONY: build clean setup run list-nodes list-topics help list-all

# 设置环境变量
setup = source install/setup.bash

# 默认目标
all: build

# 编译包
build:
	colcon build

# 清理编译文件
clean:
	rm -rf build install log

# 运行节点
# 用法1: make run PKG=<包名> NODE=<节点名>
# 用法2: make run <包名>-<节点名>
run:
ifdef PKG
	$(setup) && ros2 run $(PKG) $(NODE)
else
	$(eval ARGS := $(word 2,$(MAKECMDGOALS)))
	$(eval PKG_NAME := $(word 1,$(subst -, ,$(ARGS))))
	$(eval NODE_NAME := $(word 2,$(subst -, ,$(ARGS))))
	$(setup) && ros2 run $(PKG_NAME) $(NODE_NAME)
endif

# 运行节点（短变量名方式）
# 用法: make runp p=<包名> n=<节点名>
runp:
	$(setup) && ros2 run $(p) $(n)

# 运行节点（短横线分隔方式）
# 用法: make run- <包名>-<节点名>
run-%:
	$(setup) && ros2 run $(word 1,$(subst -, ,$*)) $(word 2,$(subst -, ,$*))

# 查看节点列表
list-nodes:
	$(setup) && ros2 node list

# 查看主题列表
list-topics:
	$(setup) && ros2 topic list

# 查看服务列表
list-services:
	$(setup) && ros2 service list

# 查看参数列表
list-params:
	$(setup) && ros2 param list

# 列出所有包和节点
list-all:
	@echo "正在列出包和节点..."
	@echo "当前工作空间: $(shell pwd)"
	@echo "检查脚本权限..."
	@ls -l scripts/list_nodes.py
	@echo "运行脚本..."
	$(setup) && python3 scripts/list_nodes.py

# 帮助信息
help:
	@echo "ROS2 开发模板 Makefile"
	@echo ""
	@echo "基本命令:"
	@echo "  make build        - 编译所有包"
	@echo "  make clean        - 清理编译文件"
	@echo ""
	@echo "运行命令:"
	@echo "  make run PKG=<包名> NODE=<节点名>  - 运行指定节点"
	@echo "  make run <包名>-<节点名>            - 运行指定节点（简写版）"
	@echo "  例如: make run basic_communication-publisher"
	@echo ""
	@echo "查看命令:"
	@echo "  make list-nodes    - 查看所有节点"
	@echo "  make list-topics   - 查看所有主题"
	@echo "  make list-services - 查看所有服务"
	@echo "  make list-params   - 查看所有参数"
	@echo "  make list-all      - 查看所有包和节点" 