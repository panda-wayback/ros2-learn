.PHONY: all clean run_turtle build run

# 默认目标
all: run

# 启动小乌龟模拟器
run_turtle:
	@echo "启动小乌龟模拟器..."
	bash -c "source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node" &

# 编译代码
build:
	@echo "编译代码..."
	@if [ ! -f "src/my_first_py_pkg/resource/my_first_py_pkg" ]; then \
		mkdir -p src/my_first_py_pkg/resource; \
		touch src/my_first_py_pkg/resource/my_first_py_pkg; \
	fi
	bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select my_first_py_pkg"

# 清理小乌龟进程
clean:
	@echo "清理小乌龟进程..."
	-@pkill -f turtlesim_node 2>/dev/null || true

# 运行新代码
run_controller:
	@echo "运行控制器..."
	bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run my_first_py_pkg turtle_controller"

# 完整流程：清理 -> 启动小乌龟 -> 编译 -> 运行控制器
run: clean run_turtle
	@sleep 2
	@$(MAKE) build
	@sleep 1
	@$(MAKE) run_controller
