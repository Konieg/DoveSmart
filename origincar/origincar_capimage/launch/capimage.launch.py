from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',     # 替换为摄像头驱动程序的包名
            executable='v4l2_camera_node,  # 替换为摄像头节点的可执行文件名
            name='v4l2_camera_node',          # 摄像头节点的名称
            output='screen',        # 输出日志到屏幕
            parameters=[
                {"resolution_width": LaunchConfiguration('resolution_width')},  # 设置摄像头分辨率宽度参数
                {"resolution_height": LaunchConfiguration('resolution_height')},  # 设置摄像头分辨率高度参数
                {"frame_rate": LaunchConfiguration('frame_rate')},  # 设置摄像头帧率参数
            ],
        ),
        Node(
            package='origincar_capimage',               # 替换为保存图像的节点的包名
            executable='capimage',  # 替换为保存图像的节点的可执行文件名
            name='capimage',  # 图像保存节点的名称
            output='screen',  # 输出日志到屏幕
            parameters=[
                {"save_directory": LaunchConfiguration('save_directory')},  # 设置图像保存目录参数
            ],
        ),
    ])
