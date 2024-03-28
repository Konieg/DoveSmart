import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture')
        self.subscription = self.create_subscription(Image, 'image_bgr', self.image_callback, 10)
        self.timer = self.create_timer(5.0, self.save_image)  # 5秒触发一次定时器
        self.cv_bridge = CvBridge()
        self.image_count = 0
        self.save_directory = 'image_cap'  # 图像保存目录
        self.cv_image = None
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

    def image_callback(self, msg):
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def save_image(self):
        if self.cv_image is not None:
            image_filename = os.path.join(self.save_directory, f'image_{self.image_count}.jpg')
            cv2.imwrite(image_filename, self.cv_image)
            self.image_count += 1
            self.get_logger().info(f'Saved {image_filename}')
        else:
            self.get_logger().warn("No image received!")

def main():
    rclpy.init()
    node = ImageCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
