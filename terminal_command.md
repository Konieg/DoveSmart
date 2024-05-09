# DoveSmart TJU

## OpenCV巡线

```Shell
ros2 launch origincar_base origincar_bringup.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run origincar_linefollower follower
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video8" -p image_size:=[640,480]
```

## 深度学习巡线

```Shell
ros2 launch origincar_base origincar_bringup.launch.py
ros2 launch origincar_bringup camera.launch.py enable_nv12_node:=true
ros2 run line_follower_resnet line_follower_resnet --ros-args -p model_path:=model/resnet18_224x224_nv12.bin -p model_name:=resnet18_224x224_nv12
```

## 识别二维码

```Shell
订阅的是/image_raw/compressed
ros2 launch qr_code_detection qr_code_detection.launch.py
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

```Shell
cd ~/dev_ws
colcon build `编译所有功能包`
colcon build --packages-select <name-of-pkg> `编译<name-of-pkg>功能包及其依赖`
colcon build --symlink-install `允许更改src中的python脚本后不再重新编译`
```

## 编译完刷新

```Shell
source /root/dev_ws/install/local_setup.bash
echo "source /root/dev_ws/install/local_setup.bash" >> ~/.bashrc
```

## 开摄像头

```Shell
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video8" -p image_size:=[640,480]
或者
ros2 launch origincar_bringup camera.launch.py <enable_nv12_node:=true> <enable_jpeg_node:=true> 
```

## 保存图像或视频

```Shell
单独运行，自带打开摄像头，不需要提前打开
ros2 launch origincar_bringup capimage <image_dir:='/root/dev_ws/src/origincar/origincar_capimage/images'> <image_width:=640> <image_height:=480> <topic_name:=/image_raw> <frequency:=0.5>
ros2 launch origincar_bringup capvideo <folder_name:='/root/dev_ws/src/origincar/origincar_capimage/videos'> <image_width:=640> <image_height:=480> <video_name:=xxx.mp4>
```

## 查看发布的话题

```Shell
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic hz /image_raw
```

## yolo障碍物检测

```Shell
ros2 launch origincar_bringup usb_websocket_display.launch.py
```

### 单张图片处理

```Shell
cd /app/pydev_demo/07_yolov5_sample/
sudo python3 ./test_yolov5.py
```
