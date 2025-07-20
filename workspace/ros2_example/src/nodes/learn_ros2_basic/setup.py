from setuptools import find_packages, setup

package_name = 'learn_ros2_basic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
            entry_points={
            'console_scripts': [
                # 01 - Simple Node
                "01_simple_node = learn_ros2_basic.01_simple_node.main:main",
                # 02 - Topic
                "02_publisher_node = learn_ros2_basic.02_topic_message.publisher_node:main",
                "02_subscriber_node = learn_ros2_basic.02_topic_message.subscriber_node:main",
                # 03 - Service
                "03_service_server = learn_ros2_basic.03_service_example.service_server:main",
                "03_service_client = learn_ros2_basic.03_service_example.service_client:main",
                "03_interactive_client = learn_ros2_basic.03_service_example.interactive_client:main",
                # 04 - Action
                "04_simple_action_server = learn_ros2_basic.04_action_example.simple_action_server:main",
                "04_simple_action_client = learn_ros2_basic.04_action_example.simple_action_client:main",
                "04_robot_navigation_example = learn_ros2_basic.04_action_example.robot_navigation_example:main",
                # 05 - Cancelable Action
                "05_simple_cancelable_server = learn_ros2_basic.05_cancelable_action_example.cancelable_action_server:main",
                "05_simple_cancelable_client = learn_ros2_basic.05_cancelable_action_example.cancelable_action_client:main",
            ],
        },
)
