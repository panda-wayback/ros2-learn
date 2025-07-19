from setuptools import find_packages, setup

package_name = 'learn_ros2'

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
                "01_simple_node = learn_ros2.01_simple_node.main:main",
                "publisher_node = learn_ros2.02_topic_message.publisher_node:main",
                "subscriber_node = learn_ros2.02_topic_message.subscriber_node:main",
                "service_server = learn_ros2.03_service_example.service_server:main",
                "service_client = learn_ros2.03_service_example.service_client:main",
                "interactive_client = learn_ros2.03_service_example.interactive_client:main",
                "simple_action_server = learn_ros2.04_action_example.simple_action_server:main",
                "simple_action_client = learn_ros2.04_action_example.simple_action_client:main",
                "robot_navigation_example = learn_ros2.04_action_example.robot_navigation_example:main",
            ],
        },
)
