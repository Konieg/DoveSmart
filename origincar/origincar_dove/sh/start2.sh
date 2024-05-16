#!/bin/bash

# 定义启动命令的字符串，确保与 start1.sh 中一致

PID=$(pgrep -f "ros2 launch origincar_base origincar_bringup.launch.py")
kill -9 $PID


# # Step 1: Stop ros2 run origincar_linefollower follower
# PID=$(pgrep -f "ros2 run origincar_linefollower follower")
# if [ -n "$PID" ]; then
#   echo "Stopping ros2 run origincar_linefollower follower with PID $PID"
#   kill -9 $PID
# fi

# # Step 2: Stop ros2 run play_football play_football
# PID=$(pgrep -f "ros2 run play_football play_football")
# if [ -n "$PID" ]; then
#   echo "Stopping ros2 run play_football play_football with PID $PID"
#   kill -9 $PID
# fi

# # Step 3: Stop ros2 launch qr_code_detection.launch.py
# PID=$(pgrep -f "ros2 launch qr_code_detection.launch.py")
# if [ -n "$PID" ]; then
#   echo "Stopping ros2 launch qr_code_detection.launch.py with PID $PID"
#   kill -9 $PID
# fi
