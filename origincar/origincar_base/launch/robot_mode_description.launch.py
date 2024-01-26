import os
from pathlib import Path
import launch_ros.actions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,LogInfo,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    origincar_description = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('origincar_description'),'urdf','origincar.urdf')]
        )
    ])
 
    ld = LaunchDescription()

    ld.add_action(origincar_description)
    return ld

