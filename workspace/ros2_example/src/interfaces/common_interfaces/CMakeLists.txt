# 设置CMake最低版本要求
# 3.8版本支持现代CMake特性，如target_include_directories等
cmake_minimum_required(VERSION 3.8)

# 定义项目名称
# 这个名称会用于生成库文件和包名
project(common_interfaces)

# 基础依赖
# 1. ament_cmake: ROS2的CMake构建系统
# 2. rosidl_default_generators: 用于生成接口代码
# 3. builtin_interfaces: 基础接口包，提供Time等基础类型
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 获取当前目录的绝对路径
# 用于后续生成相对路径的接口文件列表
# 这样可以确保rosidl_generate_interfaces使用正确的路径格式
get_filename_component(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)

# 自动查找所有接口文件（使用相对路径）
# 1. GLOB_RECURSE: 递归查找所有匹配的文件
# 2. RELATIVE: 生成相对于CURRENT_DIR的路径
# 3. 分别查找msg、srv、action三种类型的接口文件
file(GLOB_RECURSE msg_files RELATIVE ${CURRENT_DIR} "msg/*.msg")
file(GLOB_RECURSE srv_files RELATIVE ${CURRENT_DIR} "srv/*.srv")
file(GLOB_RECURSE action_files RELATIVE ${CURRENT_DIR} "action/*.action")

# 生成接口代码
# 1. ${PROJECT_NAME}: 使用项目名作为包名
# 2. 传入所有找到的接口文件
# 3. DEPENDENCIES: 声明依赖的接口包
# 这会生成C++、Python等语言的接口代码
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

# 导出依赖
# 1. rosidl_default_runtime: 运行时依赖
# 2. builtin_interfaces: 基础接口依赖
# 这样其他包在使用这个接口包时能正确找到所有依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(builtin_interfaces)

# 生成包配置
# 这会生成CMake配置文件，使其他包能够找到并使用这个包
ament_package() 