# DoveSmart TJU
## temp
```shell
ros2 launch origincar_bringup camera.launch.py image_width:=1920 image_height:=1080 
```

## origincar_move_yolo
```shell
ros2 launch origincar_base origincar_bringup.launch.py
ros2 launch origincar_bringup camera.launch.py enable_nv12_node:=true
ros2 run origincar_move_yolo origincar_move_yolo
```

## Web可视化yolo识别结果
```shell
Plan A:输入指令前，务必关闭摄像头、origincar_move_yolo和play_football
ros2 launch origincar_bringup usb_websocket_display.launch.py
浏览器输入网址:192.168.1.10:8000(有线连接)或192.168.31.143:8000(无线连接)
```

## 启动底盘手动控制小车移动
```shell
ros2 launch origincar_base origincar_bringup.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
打开bridge.exe，注意同目录下.yaml文件中的IP地址需要对应，有线为192.168.1.10
点击r开启键盘监听，点击p关闭键盘监听
上下左右箭头调整车速，WASD进行移动
```

## OpenCV巡线

```Shell
ros2 launch origincar_base origincar_bringup.launch.py
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
订阅的是/image
ros2 launch qr_code_detection qr_code_detection.launch.py
```

## 当前终端的DDS的配置
```Shell
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
其中的wlan0是绑定的网卡名，一般有线连接改为eth0，无线网卡连接改为实际的网卡名
ros2 doctor --report | grep middleware
可以查看当前终端的DDS是否切换成功
```

## 设置当前终端的DOMAIN ID
```Shell
export ROS_DOMAIN_ID=16  # 设置为16或其他不与其他团队冲突的值，仅对当前终端生效
可以在~/.bashrc文件中添加以下行，以便每次启动终端时自动设置
export ROS_DOMAIN_ID=16
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
source ~/.bashrc
```

## 开摄像头

```Shell
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video8" -p image_size:=[640,480]
或者
ros2 launch origincar_bringup camera.launch.py enable_nv12_node:=true enable_jpeg_node:=true
```

## 保存图像或视频

```Shell
单独运行，自带打开摄像头，不需要提前打开
ros2 run origincar_bringup capimage --ros-args -p image_dir:=/root/dev_ws/src/origincar/origincar_capimage/images -p frequency:=2.0 
 -p image_width:=640 -p image_height:=480 -p topic_name:=/image_raw
ros2 launch origincar_bringup capvideo <folder_name:='/root/dev_ws/src/origincar/origincar_capimage/videos'> <image_width:=640> <image_height:=480> <video_name:=xxx.mp4>
```

## 查看发布的话题

```Shell
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic hz /image_raw
```

### 单张图片处理

```Shell
cd /app/pydev_demo/07_yolov5_sample/
sudo python3 ./test_yolov5.py
```
