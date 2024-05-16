#!/bin/bash

# 使用 pgrep 检查指定的 launch 文件是否正在运行
if pgrep -f "ros2 launch origincar_base origincar_bringup.launch.py" > /dev/null; then
    echo "The launch file is already running."
else
    echo "The launch file is not running. Starting now..."
    ros2 launch origincar_base origincar_bringup.launch.py
fi


# Launch the camera with specific nodes enabled in background
# ros2 launch origincar_bringup camera.launch.py enable_nv12_node:=true enable_jpeg_node:=true &

# Start the line follower application in background


# Start the play football application in background
# ros2 run play_football play_football &

# Launch QR code detection in background
# ros2 launch qr_code_detection.launch.py &
