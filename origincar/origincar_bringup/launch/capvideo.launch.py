import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # Define launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'folder_name',
            default_value='/root/dev_ws/src/origincar/origincar_capimage/videos/',
            description='Folder to save the video file'
        ),
        DeclareLaunchArgument(
            'video_name',
            default_value='output_video.mp4',
            description='Name of the video file'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Width of the camera image'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Height of the camera image'
        )
    ]

    # Camera node configuration
    camera_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='hobot_usb_cam',
        parameters=[
            {"camera_calibration_file_path": "/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml"},
            {"frame_id": "default_usb_cam"},
            {"framerate": 25},
            {'image_width': LaunchConfiguration('image_width')},
            {'image_height': LaunchConfiguration('image_height')},
            {"io_method": "mmap"},
            {"pixel_format": "mjpeg"},
            {"video_device": "/dev/video8"},
            {"zero_copy": False}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    
    # codec node configuration
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
    
    # ImageSubscriber node configuration
    capvideo_node = Node(
        package='origincar_capimage',
        executable='capvideo',
        name='capvideo_node',
        parameters=[
            {"folder_name": LaunchConfiguration('folder_name')},
            {"video_name": LaunchConfiguration('video_name')},
            {"width": LaunchConfiguration('image_width')},
            {"height": LaunchConfiguration('image_height')}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription(
        launch_args + [camera_node, bgr8_codec_node, capvideo_node]
    )
    