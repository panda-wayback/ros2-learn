from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动发布者节点
        Node(
            package='example_pkg',
            executable='publisher',
            name='publisher_node',
            output='screen'
        ),
        # 启动订阅者节点
        Node(
            package='example_pkg',
            executable='subscriber',
            name='subscriber_node',
            output='screen'
        ),
        # 启动服务节点
        Node(
            package='example_pkg',
            executable='service',
            name='service_node',
            output='screen'
        ),
        # 启动服务客户端节点
        Node(
            package='example_pkg',
            executable='service_client',
            name='service_client_node',
            output='screen'
        ),
    ]) 