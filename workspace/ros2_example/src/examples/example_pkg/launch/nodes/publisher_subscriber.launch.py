from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_pkg',
            executable='publisher',
            name='publisher_node',
            output='screen',
            parameters=[{
                'publish_frequency': 1.0,
                'message_prefix': 'Hello from publisher'
            }]
        ),
        Node(
            package='example_pkg',
            executable='subscriber',
            name='subscriber_node',
            output='screen'
        )
    ]) 