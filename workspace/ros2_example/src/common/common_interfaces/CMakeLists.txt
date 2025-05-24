cmake_minimum_required(VERSION 3.8)
project(common_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 设置接口文件目录
set(msg_files
  "msg/Status.msg"
  "msg/Command.msg"
)

set(srv_files
  "srv/GetStatus.srv"
  "srv/SetCommand.srv"
)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

# 安装头文件
ament_export_include_directories(include)

# 安装接口文件
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(builtin_interfaces)

ament_package() 