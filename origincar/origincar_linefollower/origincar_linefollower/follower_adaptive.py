#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy, cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info("Start line follower.")

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 20)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 20)
        self.pub = self.create_publisher(Image, 'process_image', 20)

        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 转化为灰度图
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 应用自适应阈值分割
        max_value = 255
        adaptive_method = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
        threshold_type = cv2.THRESH_BINARY
        block_size = 11
        C = 2
        adaptive_thresh_image = cv2.adaptiveThreshold(gray_image, max_value, adaptive_method, threshold_type, block_size, C)

        h, w, d = image.shape
        search_top = int(h/2 + 100)
        search_bot = int(h/2 + 200)
        adaptive_thresh_image[0:search_top, 0:w] = 0
        adaptive_thresh_image[search_bot:h, 0:w] = 0

        M = cv2.moments(adaptive_thresh_image)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(image, (cx, cy), 10, (0,0,255), -1)  # 在原始彩色图中标记中心点

            # 基于检测的目标中心点，计算机器人的控制参数
            err = cx - w/2
            self.twist.linear.x = 0.1
            self.twist.angular.z = -float(err) / 400
            self.cmd_vel_pub.publish(self.twist)

        self.pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)    
    follower = Follower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
