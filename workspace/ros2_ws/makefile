.PHONY: all build run turtle controller

# ROS2 相关命令
ROS_SOURCE = source /opt/ros/humble/setup.bash
COLCON_BUILD = $(ROS_SOURCE) && colcon build
ROS_RUN = $(ROS_SOURCE) && ros2 run

# 包名和节点名
PKG_NAME = my_first_py_pkg
TURTLE_NODE = turtlesim turtlesim_node
CONTROLLER_NODE = $(PKG_NAME) turtle_controller

# 构建命令
build:
	@echo "创建构建目录..."
	@mkdir -p build install log
	@echo "编译代码..."
	@bash -c "$(COLCON_BUILD) --packages-select $(PKG_NAME)"
	@echo "构建完成"

# 运行小乌龟模拟器
turtle:
	@echo "启动小乌龟模拟器..."
	@bash -c "$(ROS_RUN) $(TURTLE_NODE)"

# 运行控制器
controller:
	@echo "启动乌龟控制器..."
	@bash -c "source install/setup.bash && $(ROS_RUN) $(CONTROLLER_NODE)"

# 运行整个系统
run: build
	@echo "启动整个系统..."
	@bash -c "$(ROS_RUN) $(TURTLE_NODE)" &
	@sleep 2
	@bash -c "source install/setup.bash && $(ROS_RUN) $(CONTROLLER_NODE)"
