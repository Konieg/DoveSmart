#!/usr/bin/python3

import os
import launch
import launch_ros
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'origincar_base'
    ld =  launch.LaunchDescription()
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name) 
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

      # Start robot localization using an Extended Kalman filter
    robot_localization_node = Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      remappings=[("odometry/filtered", "odom_combined")],
      parameters=[
          robot_localization_file_path,
          {'use_sim_time': use_sim_time}
      ])

    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to disable use_sim_time'))
    ld.add_action(robot_localization_node)
    return ld
