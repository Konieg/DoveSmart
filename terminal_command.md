#

## 巡线

```Shell
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run origincar_linefollower follower
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video8" -p image_size:=[640,480]
```

## 零拷贝

```Shell
echo performance > /sys/class/devfreq/devfreq0/governor
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
```

## 按键控制

```Shell
ros2 launch origincar_base origincar_bringup.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 编译

```
cd ~/dev_ws
colcon build `编译所有功能包`
colcon build --packages-up-to <name-of-pkg> `编译<name-of-pkg>功能包及其依赖`
colcon build --symlink-install == 允许更改src中的python脚本后不再重新编译==
```

## 编译完刷新

```Shell
source /root/dev_ws/install/local_setup.bash
echo "source /root/dev_ws/install/local_setup.bash" >> ~/.bashrc
```

## 开摄像头

```Shell
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video8" -p image_size:=[640,480]
```

## 保存图像

```Shell
ros2 run origincar_capimage capimage
```

## 查看发布的话题

```Shell
ros2 topic echo <topic_name>
ros2 topic hz /image_raw
```
