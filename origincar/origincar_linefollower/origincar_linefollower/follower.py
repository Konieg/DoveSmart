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

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(Image, '/process_image', 10)

        self.twist = Twist()
        
    def qrcode_callback(self, msg):
        if msg.data == 'ClockWise' or msg.data =='AntiClockWise':
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.get_logger().info('Shutting down node due to completion of task.')
            rclpy.shutdown()
            
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # lower_yellow = numpy.array([ 10,  70, 30])
        # upper_yellow = numpy.array([255, 255, 250])

        lower_black = numpy.array([0, 0, 0])
        upper_black = numpy.array([30, 30, 30])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        h, w, d = image.shape
        search_top = int(h/2 + 100)
        search_bot = int(h/2 + 250)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # 基于检测的目标中心点，计算机器人的控制参数
            err = cx - w/2
            self.twist.linear.x = 0.25
            self.twist.angular.z = -float(err) / 150
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
