from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'), glob('my_robot_controller/action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 Package: my_robot_controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_jump_server = my_robot_controller.turtle_jump_server:main',
            'param_example = my_robot_controller.param_example:main',
            'turtle_move_server = my_robot_controller.turtle_move_server:main',
            'turtle_move_client = my_robot_controller.turtle_move_client:main',
        ],
    },
)
