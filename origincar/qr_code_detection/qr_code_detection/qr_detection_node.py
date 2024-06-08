#!/usr/bin/env python3
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

import rclpy
import cv2
import cv_bridge
import os
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import numpy as np
from origincar_msg.msg import Sign  # 导入自定义消息
from ament_index_python.packages import get_package_share_directory

#预定义变量
color = (0, 0, 255)
thick = 3
font_scale = 0.5
font_thickness = 2


class QrCodeDetection(Node):
    def __init__(self):
        super().__init__('qrcode_detect')
        self.get_logger().info("Start qrcode_detect.")
        # 创建发布者，话题名为 /sign_switch，消息类型为 Sign
        self.pub_qrcode_info = self.create_publisher(Sign, '/sign_switch', 10)
        self.image_sub = self.create_subscription(CompressedImage, "/image", self.image_callback, 10)
        
        self.bridge = cv_bridge.CvBridge()
        self.Signmsg = Sign()
        """
        self.pub_img = self.create_publisher(
            CompressedImage, "/qrcode_detected/img_result", 10)

        self.pub_qrcode_pose = self.create_publisher(
            Pose, "/qrcode_detected/pose_result", 1)
        self.pose_result = Pose()
        """

        modelPath = os.path.join(get_package_share_directory('qr_code_detection'), 'model/') # model路径
        # self.get_logger().info('path:"{}"'.format(modelPath))

        self.detect_obj = cv2.wechat_qrcode_WeChatQRCode(
            modelPath+'detect.prototxt', modelPath+'detect.caffemodel',
            modelPath+'sr.prototxt', modelPath+'sr.caffemodel')

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

        qrInfo, qrPoints = self.detect_obj.detectAndDecode(cv_image)
        emptyList = ()
        if qrInfo != emptyList:
            self.get_logger().info('qrInfo: "{0}"'.format(qrInfo))
            self.get_logger().info('qrPoints: "{0}"'.format(qrPoints))

            qrInfo_str = qrInfo[0]
            
            if qrInfo_str == "ClockWise":
                self.Signmsg.sign_data = 3
            elif qrInfo_str == "AntiClockWise":
                self.Signmsg.sign_data = 4
            else:
                return  # If QR code does not match expected values, do nothing
            self.get_logger().info(f'Publishing: {self.Signmsg.sign_data}')
            self.pub_qrcode_info.publish(self.Signmsg)
            self.shutdown_node() # Shut down after publishing
    
    def shutdown_node(self):
        self.get_logger().info('Shutting down node after QR code detection and publish.')
        rclpy.shutdown()
        
        """
            # 获取qrPoints四个点坐标
            points = qrPoints[0]
            points_array = np.array(points, dtype=np.float32)
            # 计算四边形的中心点
            M = cv2.moments(points_array)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            self.get_logger().info('center:({},{})'.format(cX, cY))
            # 计算四边形的面积
            area = cv2.contourArea(points_array)
            self.get_logger().info('area:{}'.format(area))

            self.pose_result.position.x = float(cX)
            self.pose_result.position.y = float(cY)
            self.pose_result.position.z = float(area)
            self.pub_qrcode_pose.publish(self.pose_result)

            for pos in qrPoints:
                for p in [(0, 1), (1, 2), (2, 3), (3, 0)]:
                    start = int(pos[p[0]][0]), int(pos[p[0]][1])
                    end = int(pos[p[1]][0]), int(pos[p[1]][1])
                    cv2.line(cv_image, start, end, color, thick)

                font = cv2.FONT_HERSHEY_SIMPLEX
                text_position = (int(pos[0][0]), int(pos[0][1]) - 10)

                cv2.putText(cv_image, qrInfo_str,
                            text_position, font, font_scale, color, font_thickness)
                cv2.circle(cv_image, (cX, cY), 3, color, -1)

        # self.pub_img.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        self.pub_img.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image))
        """


def main(args=None):
    rclpy.init(args=args)
    qrCodeDetection = QrCodeDetection()
    rclpy.spin(qrCodeDetection)  # Changed to a single call of rclpy.spin

if __name__ == '__main__':
    main()
