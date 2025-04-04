   # 1. 进入容器
   docker exec -it ros2-dev bash
   
   # 2. 设置环境
   source /opt/ros/humble/setup.bash
   cd /workspace/ros2_ws
   source install/setup.bash
   
   # 3. 开发代码（在 VSCode 中）
   
   # 4. 编译
   colcon build
   
   # 5. 运行节点
   ros2 run your_package your_node