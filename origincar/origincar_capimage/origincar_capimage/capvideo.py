import os
import sys
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('folder_name', '/root/dev_ws/src/origincar/origincar_capimage2/videos/'),
                ('video_name', 'output_video.mp4'),
                ('width', 640),
                ('height', 480)
            ]
        )
        folder_name = self.get_parameter('folder_name').get_parameter_value().string_value
        video_name = self.get_parameter('video_name').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        full_video_path = os.path.expanduser(os.path.join(folder_name, video_name))
        self.get_logger().info("Start capvideo. Video path: " + full_video_path)
        self.video_writer = cv2.VideoWriter(full_video_path, cv2.VideoWriter_fourcc(*'mp4v'), 25, (width, height))
        if not self.video_writer.isOpened():
            self.get_logger().error("Failed to open video writer.")

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Received image message with timestamp: {}".format(msg.header.stamp))
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.video_writer.write(cv_image)

    def destroy_node(self):
        self.video_writer.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def handle_signal(signal, frame, context):
    print("Signal received, shutting down.")
    rclpy.shutdown(context=context)

def main(args=None):
    # Setting up argparse to parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 Image Subscriber that records video.')
    parser.add_argument('--folder_name', type=str)
    parser.add_argument('--video_name', type=str)
    parser.add_argument('--width', type=int)
    parser.add_argument('--height', type=int)
    parsed_args, unknown = parser.parse_known_args(args)

    rclpy.init(args=unknown)
    node = ImageSubscriber()

    # Override default parameters with command line arguments if provided
    if parsed_args.folder_name:
        node.set_parameter(rclpy.parameter.Parameter('folder_name', rclpy.Parameter.Type.STRING, parsed_args.folder_name))
    if parsed_args.video_name:
        node.set_parameter(rclpy.parameter.Parameter('video_name', rclpy.Parameter.Type.STRING, parsed_args.video_name))
    if parsed_args.width:
        node.set_parameter(rclpy.parameter.Parameter('width', rclpy.Parameter.Type.INTEGER, parsed_args.width))
    if parsed_args.height:
        node.set_parameter(rclpy.parameter.Parameter('height', rclpy.Parameter.Type.INTEGER, parsed_args.height))

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    signal.signal(signal.SIGINT, lambda sig, frame: handle_signal(sig, frame, node.context))
    signal.signal(signal.SIGTERM, lambda sig, frame: handle_signal(sig, frame, node.context))

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
