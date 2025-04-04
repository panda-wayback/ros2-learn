from setuptools import setup, find_namespace_packages

package_name = 'my_first_py_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_namespace_packages(include=[package_name + '*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panda',
    maintainer_email='panda@todo.todo',
    description='ROS2 turtle control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller = my_first_py_pkg.turtle_controller_node:main',
        ],
    },
)
