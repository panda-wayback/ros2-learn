# Rclcpp：节点和组件

## 创建组件


```cpp
#include <rclcpp/rclcpp.hpp>

namespace my_pkg
{

class MyComponent : public rclcpp::Node
{
public:
  MyComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("node_name", options)
  {
    // Note: you cannot use shared_from_this()
    //       here because the node is not fully
    //       initialized.
  }
};

}  // namespace my_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_pkg::MyComponent)
```

CMakeLists.txt 中的：


```cmake
add_library(my_component SHARED
  src/my_component.cpp
)
ament_target_dependencies(my_component
  rclcpp
  rclcpp_components
)

# Also add a node executable which simply loads the component
rclcpp_components_register_node(my_component
  PLUGIN "my_pkg::MyComponent"
  EXECUTABLE my_node
)
```

## Executors

要在线程中运行执行器，请执行以下操作：


```cpp
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

rclcpp::executors::SingleThreadedExecutor executor;

// Node is rclcpp::Node::SharedPtr instance
executor.add_node(node);
std::thread executor_thread(
  std::bind(&rclcpp::executors::SingleThreadedExecutor::spin,
            &executor));
```
