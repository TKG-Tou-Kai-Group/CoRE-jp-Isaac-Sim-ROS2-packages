import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import os

class ImageOverlayPublisher(Node):
    def __init__(self):
        super().__init__('image_overlay_publisher')
        self.subscription = self.create_subscription(
            Image,
            'input_image_topic',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(CompressedImage, 'output_image_topic', 10)
        self.bridge = CvBridge()

        package_share_directory = get_package_share_directory('core_jp_camera_publisher')
        overlay_image_path = os.path.join(package_share_directory, 'data', 'Reticle.png')
        self.overlay = cv2.imread(overlay_image_path, cv2.IMREAD_UNCHANGED)  # 透過PNGを読み込む

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        output_image = self.overlay_image(cv_image, self.overlay)
        compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(output_image, dst_format='jpg')
        self.publisher.publish(compressed_image_msg)

    def overlay_image(self, background, overlay):
        overlay_height, overlay_width = overlay.shape[:2]
        bg_height, bg_width = background.shape[:2]

        # サイズを背景画像に合わせて変更
        if overlay_height > bg_height or overlay_width > bg_width:
            scale = min(bg_width / overlay_width, bg_height / overlay_height)
            overlay = cv2.resize(overlay, (int(overlay_width * scale), int(overlay_height * scale)), interpolation=cv2.INTER_AREA)

        # 透過PNGのチャンネル分離
        b, g, r, a = cv2.split(overlay)
        overlay_rgb = cv2.merge((b, g, r))

        # マスク作成
        alpha = a / 255.0
        alpha_inv = 1.0 - alpha

        # 合成位置の指定
        y1, x1 = 0, 0
        y2, x2 = y1 + overlay.shape[0], x1 + overlay.shape[1]

        # 合成
        for c in range(0, 3):
            background[y1:y2, x1:x2, c] = (alpha * overlay_rgb[:, :, c] + alpha_inv * background[y1:y2, x1:x2, c])

        return background

def main(args=None):
    rclpy.init(args=args)
    node = ImageOverlayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

