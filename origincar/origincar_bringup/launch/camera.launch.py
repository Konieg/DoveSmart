import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Define common launch arguments
    # 声明命令行参数
    launch_args = [
        DeclareLaunchArgument('device', default_value='/dev/video8', description='usb camera device'),
        DeclareLaunchArgument('image_width', default_value='640', description='Width of the camera image'),
        DeclareLaunchArgument('image_height', default_value='480', description='Height of the camera image'),
        DeclareLaunchArgument('enable_nv12_node', default_value='false', description='Enable NV12 codec node'),
        DeclareLaunchArgument('enable_jpeg_node', default_value='false', description='Enable JPEG Node')
    ]
    
    # Camera node configuration
    # 启动hobot-usb-cam，发布CompressedImage类型的/image话题
    camera_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='hobot_usb_cam',
        parameters=[
            {"camera_calibration_file_path": "/opt/tros/lib/hobot_usb_cam/config/DoveSmart-640-480.yaml"},
            {"frame_id": "default_usb_cam"},
            {"framerate": 30},
            {'image_width': LaunchConfiguration('image_width')},
            {'image_height': LaunchConfiguration('image_height')},
            {"io_method": "mmap"},
            {"pixel_format": "mjpeg"},
            {"video_device": LaunchConfiguration('device')},
            {"zero_copy": False}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    
    # bgr8 Codec node configuration
    # 启动hobot-codec-decode，发布Image类型的/image_raw话题
    bgr8_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"channel": 1},
            {"in_mode": "ros"},
            {"in_format": "jpeg"},
            {"out_mode": "ros"},
            {"out_format": "bgr8"},
            {"sub_topic": "/image"},
            {"pub_topic": "/image_raw"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    
    # Conditional inclusion of nv12_codec_node based on launch argument
    # 使用hobot-codec-decode，发布nv12类型的/hbmem_img话题，用于深度学习
    enable_nv12_node = LaunchConfiguration('enable_nv12_node')
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_out_format': 'nv12',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items(),
        condition=IfCondition(enable_nv12_node)
    )
    
    # jpeg node configuration
    # jpeg_codec_node使用hobot-codec-encode，发布CompressedImage类型的/image_raw/compressed话题
    enable_jpeg_node = LaunchConfiguration('enable_jpeg_node')
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_in_format': 'bgr8',
            'codec_out_format': 'jpeg',
            'codec_sub_topic': '/image_raw',
            'codec_pub_topic': '/image_raw/compressed'
        }.items(),
        condition=IfCondition(enable_jpeg_node)
    )
    
    # jpeg_node使用relay_img，将Image类型的/image话题转发为CompressedImage类型的/image_raw/compressed话题
    # 因为现在hobot-usb-cam包已经更新，/image话题是CompressedImage类型，所以不需要再转发
    jpeg_node = Node(
        package='origincar_demo',
        executable='relay_img',
        name='relay_img',
        output='screen',
        parameters=[],
        arguments=['--ros-args', '--log-level', 'error'],
        condition=IfCondition(enable_jpeg_node)  
    )

    # Return the launch description, conditionally including the nv12_codec_node
    return LaunchDescription(
        launch_args + [camera_node, bgr8_codec_node, nv12_codec_node]
    )
    
