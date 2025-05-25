from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    use_pub_sub = LaunchConfiguration('use_pub_sub', default='true')
    use_service = LaunchConfiguration('use_service', default='true')
    
    # 获取包的路径
    pkg_dir = get_package_share_directory('example_pkg')
    
    # 创建 launch 描述
    ld = LaunchDescription([
        # 声明可配置的启动参数
        DeclareLaunchArgument(
            'use_pub_sub',
            default_value='true',
            description='是否启动发布者和订阅者节点'
        ),
        DeclareLaunchArgument(
            'use_service',
            default_value='true',
            description='是否启动服务节点'
        ),
    ])
    
    # 条件性地包含发布者/订阅者 launch 文件
    pub_sub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nodes', 'publisher_subscriber.launch.py')
        ),
        condition=LaunchConfiguration('use_pub_sub')
    )
    
    # 条件性地包含服务 launch 文件
    service_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nodes', 'service.launch.py')
        ),
        condition=LaunchConfiguration('use_service')
    )
    
    # 添加 launch 文件
    ld.add_action(pub_sub_launch)
    ld.add_action(service_launch)
    
    return ld 