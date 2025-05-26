from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_pkg',
            executable='service',
            name='service_node',
            output='screen'
        ),
        Node(
            package='example_pkg',
            executable='service_client',
            name='service_client_node',
            output='screen',
            parameters=[{
                'request_interval': 2.0
            }]
        )
    ]) 