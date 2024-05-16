from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 定义路径
    qr_code_detection_dir = get_package_share_directory('qr_code_detection')
    origincar_bringup_dir = get_package_share_directory('origincar_bringup')
    origincar_base_dir = get_package_share_directory('origincar_base')

    # 定义启动参数
    enable_nv12_node = DeclareLaunchArgument(
        'enable_nv12_node', default_value='true', description='Enable NV12 Node')

    enable_jpeg_node = DeclareLaunchArgument(
        'enable_jpeg_node', default_value='true', description='Enable JPEG Node')

    # Include其他包的启动文件
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(origincar_bringup_dir, 'camera.launch.py')),
        launch_arguments={'enable_nv12_node': LaunchConfiguration('enable_nv12_node'),
                          'enable_jpeg_node': LaunchConfiguration('enable_jpeg_node')}.items())

    qr_code_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(qr_code_detection_dir, 'qr_code_detection.launch.py')))

    origincar_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(origincar_base_dir, 'origincar_bringup.launch.py')))

    # 独立运行的节点
    line_follower_node = Node(
        package='line_follower_resnet',
        executable='line_follower_resnet',
        name='line_follower_resnet',
        parameters=[
            {'model_path': '/root/dev_ws/src/origincar/line_follower_resnet/model/resnet18_224x224_nv12.bin'},
            {'model_name': 'resnet18_224x224_nv12'}
        ],
        output='screen'
    )

    # 启动LifecycleManager节点
    lifecycle_manager_node = Node(
        package='origincar_lifecycle', 
        executable='lifecycle',  
        name='lifecycle_manager',
        output='screen'
    )

    # 组合所有节点和启动文件
    return LaunchDescription([
        enable_nv12_node,
        enable_jpeg_node,
        camera_launch,
        qr_code_detection_launch,
        line_follower_node,
        origincar_bringup_launch,
        lifecycle_manager_node  \
    ])
