from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # USB Camera Node
    usb_cam_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='usb_cam_node',
        output='screen',
        arguments=[
            '--ros-args', '--log-level', 'warn',
            '--ros-args', '-p', 'zero_copy:=False',
            '--ros-args', '-p', 'io_method:=mmap',
            '--ros-args', '-p', 'video_device:=/dev/video8',
            '--ros-args', '-p', 'pixel_format:=mjpeg',
            '--ros-args', '-p', 'image_height:=640',
            '--ros-args', '-p', 'image_width:=480'
        ]
    )

    # Codec Node
    codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        name='codec_node',
        output='screen',
        arguments=[
            '--ros-args', '--log-level', 'warn',
            '--ros-args', '-p', 'in_format:=jpeg',
            '--ros-args', '-p', 'out_format:=nv12',
            '--ros-args', '-p', 'channel:=1',
            '--ros-args', '-p', 'out_mode:=shared_mem',
            '--ros-args', '-p', 'sub_topic:=image',
            '--ros-args', '-p', 'pub_topic:=hbmem_img'
        ]
    )

     # 启动line_follower_resnet节点并设置参数
    line_follower_node = Node(
        package='line_follower_resnet',
        executable='line_follower_resnet',
        name='line_follower_resnet',
        output='screen',
        parameters=[
            {'model_path': '/root/dev_ws/src/origincar/line_follower_resnet/model/resnet18_224x224_nv12.bin'},
            {'model_name': 'resnet18_224x224_nv12'}
        ]
    )

    # Websocket Node
    websocket_node = Node(
        package='websocket',
        executable='websocket',
        name='websocket_node',
        output='screen',
        arguments=[
            '--ros-args', '--log-level', 'error',
            '--ros-args', '-p', 'only_show_image:=False',
            '--ros-args', '-p', 'image_topic:=image',
            '--ros-args', '-p', 'image_type:=mjpeg',
            '--ros-args', '-p', 'smart_topic:=playfootball_node'
        ]
    )
    
    qr_detection_node = Node(
        package='qr_code_detection',
        executable='qr_detection_node',
    )

    play_football_node = Node(
        package = 'play_football',
        executable = 'play_football',
    )
    
    return LaunchDescription([
        qr_detection_node,
        line_follower_node
    ])