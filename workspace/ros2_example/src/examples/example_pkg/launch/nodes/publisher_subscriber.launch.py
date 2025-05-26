from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的路径
    pkg_dir = get_package_share_directory('example_pkg')
    
    # 声明参数
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    
    # 创建 launch 描述
    ld = LaunchDescription([
        # 声明可配置的启动参数
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_dir, 'launch', 'config', 'publisher_params.yaml'),
            description='参数配置文件路径'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别 (debug, info, warn, error, fatal)'
        ),
    ])
    
    # 发布者节点
    publisher_node = Node(
        package='example_pkg',
        executable='publisher',
        name='publisher_node',
        output='screen',
        parameters=[
            config_file,  # 从配置文件加载参数
            {
                'use_sim_time': use_sim_time,  # 从命令行参数加载
            }
        ],
        arguments=['--ros-args', '--log-level', log_level]  # 设置日志级别
    )
    
    # 订阅者节点
    subscriber_node = Node(
        package='example_pkg',
        executable='subscriber',
        name='subscriber_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'launch', 'config', 'subscriber_params.yaml'),
            {
                'use_sim_time': use_sim_time,
            }
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # 添加节点
    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    
    return ld 