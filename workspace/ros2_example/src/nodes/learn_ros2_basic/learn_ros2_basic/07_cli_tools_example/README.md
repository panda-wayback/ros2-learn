# 示例7: ROS2命令行工具(CLI)调试

本示例旨在演示ROS2中两个最核心、最常用的命令行调试工具：
- `ros2 topic echo`
- `ros2 service call`

掌握这两个工具，可以让你在不编写额外代码的情况下，快速地检查、测试和调试正在运行的ROS2系统，极大地提高开发效率。

## 文件结构

- `cli_service_server.py`: 一个简单的ROS2节点，它同时提供一个Service和一个Publisher，专门用于配合这两个CLI工具进行演示。

### 节点行为

1.  **提供服务 (`/add_two_ints`)**: 这是一个`AddTwoInts`类型的服务，接收两个整数 `a` 和 `b`，并返回它们的和 `sum`。
2.  **发布话题 (`/calc_history`)**: 每次服务被成功调用后，它都会将计算的过程和结果（例如 `"计算历史: 5 + 10 = 15"`）发布到这个`String`类型的话题上。

## 核心概念

### 1. `ros2 topic echo <topic_name>`

-   **作用**: **窃听**一个正在广播的话题。它会实时地将该话题上流动的每一条消息打印到你的终端上。
-   **比喻**: 一个可以调到任何频道的对讲机，让你能听到该频道上的所有对话。
-   **用途**:
    -   检查一个Publisher是否在正常工作。
    -   实时查看传感器数据的内容。
    -   调试消息的格式和内容是否符合预期。

### 2. `ros2 service call <service_name> <service_type> '<request_data>'`

-   **作用**: **调用**一个正在运行的服务。它扮演一个临时的客户端，向指定的服务发送请求，并等待服务端的响应。
-   **比喻**: 直接走到餐厅前台点餐，并等待服务员把菜端回来。
-   **用途**:
    -   快速测试一个Service Server的业务逻辑是否正确。
    -   在没有客户端节点的情况下，手动触发一个服务。
    -   在开发中执行一次性的操作或请求数据。

## 🚀 如何运行和体验

请严格按照以下步骤操作，来感受这两个命令的强大之处。

### 步骤 1: 构建工作区

首先，和往常一样，构建你的包以使所有改动生效。
```bash
colcon build --packages-select learn_ros2_basic
```

### 步骤 2: 启动服务端节点

打开**第一个终端**，启动我们创建的服务端。
```bash
source install/setup.bash
ros2 run learn_ros2_basic 07_cli_service_server
```
> 你会看到它打印出 “服务端已准备就绪” 的信息，然后它会安静地等待。

### 步骤 3: 使用 `ros2 topic echo` “窃听” 话题

现在，打开**第二个终端**，输入以下命令来“窃听” `/calc_history` 话题：
```bash
source install/setup.bash
ros2 topic echo /calc_history
```
> 这个终端现在会**看起来像卡住了**，什么也不显示。**这是正常的！** 它正在等待新的消息出现。让这个终端保持运行。

### 步骤 4: 使用 `ros2 service call` 调用服务

现在，最关键的一步来了。打开**第三个终端**，我们将在这里手动调用服务。输入以下命令：

```bash
source install/setup.bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
```
> 按下回车后，你会在这个终端看到服务端的**响应**。

### 步骤 5: 观察所有终端

在你按下回车的**一瞬间**，观察所有三个终端：
-   **终端1 (服务端)**: 会打印出它收到了请求 `a=5, b=10`，并发布了历史记录。
-   **终端2 (`topic echo`)**: 会立刻打印出服务端发布的消息，内容是 `"计算历史: 5 + 10 = 15"`。
-   **终端3 (`service call`)**: 会打印出来自服务端的最终响应 `sum=15`。

### 再次尝试

你可以尝试在第三个终端用不同的参数多次调用服务，每次都会在其他两个终端看到相应的输出。
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 100, b: -50}"
```

通过这个实验，你就掌握了ROS2开发中最核心的两个调试利器！ 