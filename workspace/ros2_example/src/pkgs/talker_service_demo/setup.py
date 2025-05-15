from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'talker_service_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
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
            'talker_service = talker_service_demo.talker_service_node:main',
            'second_talker = talker_service_demo.second_talker_node:main',
            'robot_status = talker_service_demo.robot_status_node:main',
        ],
    },
)
