import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Define launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Width of the camera image'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Height of the camera image'
        ),
        DeclareLaunchArgument(
            'image_dir',
            default_value='/root/dev_ws/src/origincar/origincar_capimage/images',
            description='Directory to save images'
        ),
        DeclareLaunchArgument(
            'topic_name',
            default_value='image_raw',
            description='The ROS2 topic to subscribe for images'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='0.5',
            description='The cap frequency'
        ),
        DeclareLaunchArgument(
            'format',
            default_value='bgr8',
            description='The capimage format'
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
            {"framerate": 30},
            {'image_width': LaunchConfiguration('image_width')},
            {'image_height': LaunchConfiguration('image_height')},
            {"io_method": "mmap"},
            {"pixel_format": "mjpeg"},
            {"video_device": "/dev/video8"},
            {"zero_copy": False}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    
    # Codec node configuration
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
    
    # capimage node configuration
    capimage_node = Node(
        package='origincar_capimage',
        executable='capimage',
        name='capimage_node',
        output='screen',
        parameters=[
            {"image_dir": LaunchConfiguration('image_dir')},
            {"topic_name": LaunchConfiguration('topic_name')},
            {"width": LaunchConfiguration('image_width')},
            {"height": LaunchConfiguration('image_height')},
            {"frequency": LaunchConfiguration('frequency')},
            {"format": LaunchConfiguration('format')}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription(
        launch_args + [camera_node, bgr8_codec_node, capimage_node]
    )
