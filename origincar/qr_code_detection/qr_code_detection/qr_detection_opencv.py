#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from origincar_msg.msg import Sign  # 导入自定义消息
from ament_index_python.packages import get_package_share_directory

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        # 创建发布者，话题名为 /sign_switch，消息类型为 Sign
        self.pub_qrcode_info = self.create_publisher(Sign, '/sign_switch', 10)
        self.Signmsg = Sign()
        self.bridge = CvBridge()
        self.get_logger().info('QR Code Detector Node has been started.')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.detect_qr_code(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def detect_qr_code(self, image):
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(image)
        if bbox is not None:
            self.get_logger().info(f'Detected QR code: {data}')
            if data == "ClockWise":
                self.Signmsg.sign_data = 3
            elif data == "AntiClockWise":
                self.Signmsg.sign_data = 4
            else:
                return  # If QR code does not match expected values, do nothing
            self.get_logger().info(f'Publishing: {self.Signmsg.sign_data}')
            self.pub_qrcode_info.publish(self.Signmsg)
            self.shutdown_node() # Shut down after publishing
            
    def shutdown_node(self):
        self.get_logger().info('Shutting down node after QR code detection and publish.')
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    qr_code_detector = QRCodeDetector()
    rclpy.spin(qr_code_detector)
    qr_code_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
