import rclpy
import cv2
import cv_bridge
from rclpy.lifecycle import LifecycleNode
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os

class QrCodeDetection(LifecycleNode):
    def __init__(self):
        super().__init__('qrcode_detect')
        self.bridge = cv_bridge.CvBridge()
        self.pub_qrcode_info = None
        self.image_sub = None
        self.info_result = String()

        model_path = os.path.join(get_package_share_directory('qr_code_detection'), 'model/')
        self.detect_obj = cv2.wechat_qrcode_WeChatQRCode(
            model_path + 'detect.prototxt', model_path + 'detect.caffemodel',
            model_path + 'sr.prototxt', model_path + 'sr.caffemodel')
        self.get_logger().info("QrCodeDetection Node Created, awaiting configuration.")

    def on_configure(self, state):
        self.get_logger().info("Configuring QrCodeDetection Node.")
        self.image_sub = self.create_subscription(
            CompressedImage, "/image_raw/compressed", self.image_callback, 10)
        self.pub_qrcode_info = self.create_publisher(
            String, "/qrcode_detected/info_result", 10)
        return LifecycleNode.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("Activating QrCodeDetection Node.")
        if self.pub_qrcode_info:
            self.pub_qrcode_info.on_activate()
        return LifecycleNode.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info("Deactivating QrCodeDetection Node.")
        if self.pub_qrcode_info:
            self.pub_qrcode_info.on_deactivate()
        return LifecycleNode.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info("Cleaning up QrCodeDetection Node.")
        self.pub_qrcode_info = None
        self.image_sub = None
        return LifecycleNode.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info("Shutting down QrCodeDetection Node.")
        return LifecycleNode.SUCCESS

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            qrInfo, qrPoints = self.detect_obj.detectAndDecode(cv_image)
            if qrInfo:
                self.get_logger().info(f'qrInfo: "{qrInfo[0]}"')
                self.get_logger().info(f'qrPoints: "{qrPoints}"')
                self.info_result.data = qrInfo[0]
                self.pub_qrcode_info.publish(self.info_result)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    qr_code_detection = QrCodeDetection()

    lifecycle_executor = rclpy.executors.SingleThreadedExecutor()
    lifecycle_executor.add_node(qr_code_detection)

    try:
        lifecycle_executor.spin()
    except KeyboardInterrupt:
        qr_code_detection.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        qr_code_detection.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()