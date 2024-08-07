import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor(Node):

    def __init__(self):
        super().__init__('image_compressor')
        self.subscription = self.create_subscription(
            Image,
            'input_image_topic',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(CompressedImage, 'compressed_image_topic', 10)
        self.br = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, 'bgr8')
        
        # 画像をJPEG形式で圧縮
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 圧縮率は適宜調整
        result, encimg = cv2.imencode('.jpg', cv_image, encode_param)

        if result:
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encimg.tobytes()
            self.publisher_.publish(compressed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

