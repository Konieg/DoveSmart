import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
import numpy as np
from pathlib import Path

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_dir', '/root/dev_ws/src/origincar/origincar_capimage/images'),
                ('width', 640),
                ('height', 480),
                ('frequency', 0.5),
                ('topic_name', '/image_raw'),
                ('format', 'bgr8')  # Default format is 'bgr8'
            ]
        )
        image_dir = self.get_parameter('image_dir').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        format = self.get_parameter('format').get_parameter_value().string_value

        self.image_dir = os.path.expanduser(image_dir)
        os.makedirs(self.image_dir, exist_ok=True)
        self.get_logger().info(f"Images will be saved to: {self.image_dir}")
        self.get_logger().info(f"Topic name: {topic_name}")
        
        self.index = find_start_index(self.image_dir)
        self.resolution = (width, height)
        self.capture_frequency = frequency
        self.image_format = format

        self.subscriber = self.create_subscription(Image, topic_name, self.callback, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / self.capture_frequency, self.timer_callback)
        self.last_image = None

    def callback(self, data):
        if self.image_format == 'bgr8':
            self.last_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        elif self.image_format == 'nv12':
            if data.height * data.width * 1.5 == len(data.data):
                self.last_image = np.frombuffer(data.data, dtype=np.uint8).reshape((data.height + data.height // 2, data.width))
            else:
                self.get_logger().error("Data size does not match NV12 format expectations.")


    def timer_callback(self):
        # 图像分辨率和摄像头的分辨率相同
        if self.last_image is not None:
            resized_image = cv2.resize(self.last_image, self.resolution)
            cv2.imwrite(os.path.join(self.image_dir, f'{self.index}.jpg'), resized_image)
            self.get_logger().info(f'Saved image {self.index}.jpg')
            self.index += 1
        # 图像分辨率为[640, 224]，用于深度学习巡线
        # if self.last_image is not None:
        #     filename = os.path.join(self.image_dir, f'{self.index}.jpg')
        #     if self.image_format == 'bgr8':
        #         cropped_image = self.last_image[240:464, :]  # Crop to vertical range [240, 464]
        #         resized_image = cv2.resize(cropped_image, (640, 224))  # Specify resolution
        #         cv2.imwrite(filename, resized_image)
        #         self.get_logger().info(f'Saved bgr8 image {self.index}.jpg')
        #     elif self.image_format == 'nv12':
        #         bgr_image = cv2.cvtColor(self.last_image, cv2.COLOR_YUV2BGR_NV12)
        #         cv2.imwrite(os.path.join(self.image_dir, f'{self.index}.jpg'), bgr_image)
        #         self.get_logger().info(f'Saved nv12 image {self.index}.jpg')

        #     self.index += 1

def find_start_index(image_dir):
    images = list(Path(image_dir).glob('*.jpg'))
    if images:
        return max(int(img.stem) for img in images) + 1
    return 0

def main(args=None):
    parser = argparse.ArgumentParser(description="Save images from a ROS2 topic.")
    parser.add_argument("--image_dir", type=str)
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--frequency", type=float)
    parser.add_argument("--topic_name", type=str)
    parser.add_argument("--format", type=str, choices=['bgr8', 'nv12'], default='bgr8')

    parsed_args, unknown = parser.parse_known_args(args)

    rclpy.init(args=unknown)
    node = ImageSaver()

    # Override default parameters with command line arguments if provided
    if parsed_args.image_dir:
        node.set_parameters([rclpy.parameter.Parameter('image_dir', rclpy.Parameter.Type.STRING, parsed_args.image_dir)])
    if parsed_args.width is not None:
        node.set_parameters([rclpy.parameter.Parameter('width', rclpy.Parameter.Type.INTEGER, parsed_args.width)])
    if parsed_args.height is not None:
        node.set_parameters([rclpy.parameter.Parameter('height', rclpy.Parameter.Type.INTEGER, parsed_args.height)])
    if parsed_args.frequency:
        node.set_parameters([rclpy.parameter.Parameter('frequency', rclpy.Parameter.Type.DOUBLE, parsed_args.frequency)])
    if parsed_args.topic_name:
        node.set_parameters([rclpy.parameter.Parameter('topic_name', rclpy.Parameter.Type.STRING, parsed_args.topic_name)])
    if parsed_args.format:
        node.set_parameters([rclpy.parameter.Parameter('format', rclpy.Parameter.Type.STRING, parsed_args.format)])
        
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
