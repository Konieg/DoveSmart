# DoveSmart TJU

## 第19届全国大学生智能车竞赛（地平线智慧医疗）同济大学睿行·DoveSmart队

## 避障以及巡线(origincar_move_yolo)

```shell
ros2 launch origincar_base origincar_bringup.launch.py
ros2 launch origincar_bringup camera.launch.py enable_nv12_node:=true
ros2 run origincar_move_yolo origincar_move_yolo
```

## 开启上位机

```shell
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 识别二维码(订阅话题：/image)

```Shell
ros2 launch qr_code_detection qr_code_detection.launch.py
```

## 编译(进入dev_ws)

```Shell
colcon build --packages-select <name-of-pkg> `编译
```

## 编译完刷新

```Shell
source ~/.bashrc
```
