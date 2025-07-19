SHELL := /bin/bash

.PHONY: build clean setup run list-nodes list-topics help list-all

# 设置环境变量
setup = source install/setup.bash

# 默认目标
all: build

update-paths:
	python3 scripts/auto_update_paths.py

# 清理编译文件
clean:
	rm -rf build install log

build: clean
	colcon build

buildEx: clean build list-all

# make launch p=src/nodes/learn_ros2/learn_ros2/02_topic_message/topic_demo.launch.py
launch:
	$(setup) && ros2 launch $(p)

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
	@:
endif

# 处理带连字符的参数
%:
	@:

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


# 创建一个自定义 interface 包
# 用法: make create-interface
create-interface:
	@echo "创建新的自定义 interface 包..."
	@read -p "请输入 interface 包名: " pkg_name; \
	cd src/interfaces; \
	ros2 pkg create --build-type ament_cmake $$pkg_name
	@echo "interface 包创建完成！"
	@echo "请在 src/interfaces/$$pkg_name/ 目录下添加 msg/srv/action 文件夹和接口文件"
	@echo "然后编辑 CMakeLists.txt 和 package.xml 配置接口生成"
	@echo "最后运行 'make build' 来编译包"

# 创建一个新的 python 节点
# 在 src 目录下
create-python:
	@echo "创建新的 Python 节点包..."
	@read -p "请输入包名: " pkg_name; \
	cd src/nodes; \
	ros2 pkg create --build-type ament_python $$pkg_name
	@echo "包创建完成！"
	@echo "请编辑 src/$$pkg_name/ 目录下的文件来添加节点代码"
	@echo "然后运行 'make build' 来编译包"


# 创建一个新的 C++ 节点
# 在 src 目录下
create-cpp:
	@echo "创建新的 C++ 节点包..."
	@read -p "请输入包名: " pkg_name; \
	cd src/temp; \
	ros2 pkg create --build-type ament_cmake $$pkg_name
	@echo "包创建完成！"
	@echo "请编辑 src/$$pkg_name/src/ 目录下的文件来添加节点代码"
	@echo "然后运行 'make build' 来编译包"


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
	@echo "  make create-python - 创建一个新的 Python 节点"
	@echo "  make create-cpp    - 创建一个新的 C++ 节点"
	@echo "  make help          - 查看帮助信息"
	@echo "  make setup         - 设置环境变量"
	@echo "  make clean         - 清理编译文件"
	@echo "  make build         - 编译所有包"
	@echo "  make runp          - 运行指定节点（短变量名方式）"