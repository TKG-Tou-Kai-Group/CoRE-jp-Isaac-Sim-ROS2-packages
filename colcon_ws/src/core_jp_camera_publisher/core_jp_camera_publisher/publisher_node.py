import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32
import cv2
import numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import os
from PIL import Image as PILImage, ImageDraw, ImageFont

STATUS_NORMAL =     0
STATUS_RED_WIN =    1
STATUS_BLUE_WIN =   2
STATUS_ROBOT1_WIN = 3
STATUS_ROBOT2_WIN = 4
STATUS_ROBOT3_WIN = 5
STATUS_ROBOT4_WIN = 6

class ImageOverlayPublisher(Node):
    def __init__(self):
        super().__init__('image_overlay_publisher')
        self.subscription = self.create_subscription(
            Image,
            'input_image_topic',
            self.image_callback,
            10)
        self.status_subscription = self.create_subscription(
            Int32,
            'game_status',
            self.status_callback,
            10)
        self.time_subscription = self.create_subscription(
            Int32,
            'countdown',
            self.countdown_callback,
            10)
        self.robot1_hp_subscription = self.create_subscription(
            Int32,
            'robot1_hp',
            self.robot1_hp_callback,
            10)
        self.robot2_hp_subscription = self.create_subscription(
            Int32,
            'robot2_hp',
            self.robot2_hp_callback,
            10)
        self.robot3_hp_subscription = self.create_subscription(
            Int32,
            'robot3_hp',
            self.robot3_hp_callback,
            10)
        self.robot4_hp_subscription = self.create_subscription(
            Int32,
            'robot4_hp',
            self.robot4_hp_callback,
            10)
        self.robot5_hp_subscription = self.create_subscription(
            Int32,
            'robot5_hp',
            self.robot5_hp_callback,
            10)
        self.robot6_hp_subscription = self.create_subscription(
            Int32,
            'robot6_hp',
            self.robot6_hp_callback,
            10)
        self.robot7_hp_subscription = self.create_subscription(
            Int32,
            'robot7_hp',
            self.robot7_hp_callback,
            10)
        self.robot8_hp_subscription = self.create_subscription(
            Int32,
            'robot8_hp',
            self.robot8_hp_callback,
            10)
        self.publisher = self.create_publisher(CompressedImage, 'output_image_topic', 10)
        self.bridge = CvBridge()

        package_share_directory = get_package_share_directory('core_jp_camera_publisher')
        overlay_image_path = os.path.join(package_share_directory, 'data', 'Reticle.png')
        self.overlay = cv2.imread(overlay_image_path, cv2.IMREAD_UNCHANGED)  # 透過PNGを読み込む
        
        self.game_status = STATUS_NORMAL
        self.game_time = 0
        self.robot_hp = [0,0,0,0,0,0,0,0]
        self.max_robot_hp = [0,0,0,0,0,0,0,0]

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 画像の高さと幅を取得
        height, width, _ = cv_image.shape

        output_image = self.overlay_image(cv_image, self.overlay)
        output_image = self.draw_countdown(output_image)
        output_image = self.draw_robot_status(output_image, "sample_robot1", 1, 10, 10)
        output_image = self.draw_robot_status(output_image, "sample_robot2", 2, int(width*5/6) - 10, 10)
        output_image = self.draw_robot_status(output_image, "sample_robot3", 3, 10, int(height / 10) + 20)
        output_image = self.draw_robot_status(output_image, "sample_robot4", 4, int(width*5/6) - 10, int(height / 10) + 20)
        output_image = self.draw_robot_status(output_image, "sample_robot5", 5, 10, int(height / 10) * 2 + 30)
        output_image = self.draw_robot_status(output_image, "sample_robot6", 6, int(width*5/6) - 10, int(height / 10) * 2 + 30)
        output_image = self.draw_robot_status(output_image, "sample_robot7", 7, 10, int(height / 10) * 3 + 40)
        output_image = self.draw_robot_status(output_image, "sample_robot8", 8, int(width*5/6) - 10, int(height / 10) * 3 + 40)
        output_image = self.draw_result(output_image)
        compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(output_image, dst_format='jpg')
        self.publisher.publish(compressed_image_msg)

    def status_callback(self, msg):
        self.game_status = msg.data

    def countdown_callback(self, msg):
        self.game_time = msg.data

    def robot1_hp_callback(self, msg):
        self.robot_hp[0] = msg.data
        if self.max_robot_hp[0] < self.robot_hp[0]:
            self.max_robot_hp[0] = self.robot_hp[0]

    def robot2_hp_callback(self, msg):
        self.robot_hp[1] = msg.data
        if self.max_robot_hp[1] < self.robot_hp[1]:
            self.max_robot_hp[1] = self.robot_hp[1]

    def robot3_hp_callback(self, msg):
        self.robot_hp[2] = msg.data
        if self.max_robot_hp[2] < self.robot_hp[2]:
            self.max_robot_hp[2] = self.robot_hp[2]

    def robot4_hp_callback(self, msg):
        self.robot_hp[3] = msg.data
        if self.max_robot_hp[3] < self.robot_hp[3]:
            self.max_robot_hp[3] = self.robot_hp[3]

    def robot5_hp_callback(self, msg):
        self.robot_hp[4] = msg.data
        if self.max_robot_hp[4] < self.robot_hp[4]:
            self.max_robot_hp[4] = self.robot_hp[4]

    def robot6_hp_callback(self, msg):
        self.robot_hp[5] = msg.data
        if self.max_robot_hp[5] < self.robot_hp[5]:
            self.max_robot_hp[5] = self.robot_hp[5]

    def robot7_hp_callback(self, msg):
        self.robot_hp[6] = msg.data
        if self.max_robot_hp[6] < self.robot_hp[6]:
            self.max_robot_hp[6] = self.robot_hp[6]

    def robot8_hp_callback(self, msg):
        self.robot_hp[7] = msg.data
        if self.max_robot_hp[7] < self.robot_hp[7]:
            self.max_robot_hp[7] = self.robot_hp[7]

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

    def draw_result(self, image):
        if self.game_time < 0:
            # OpenCV画像をPillow画像に変換
            cv_image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(cv_image_rgb)

            # 画像の高さと幅を取得
            width, height = pil_image.size
            
            draw = ImageDraw.Draw(pil_image)

            # フォントの設定
            font_size = int(height // 8)
            font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
            font = ImageFont.truetype(font_path, font_size)

            # テキストの描画位置を計算
            text_width, text_height = draw.textsize(str(-self.game_time), font=font)
            text_x = (width - text_width) // 2
            text_y = (height - text_height) // 2

            # 四角形内にテキストを描画
            draw.text((text_x, text_y), str(-self.game_time), font=font, fill=(255, 255, 255))

            # Pillow画像をOpenCV画像に戻す
            cv_image = np.array(pil_image)
            image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        
        elif not self.game_status == 0:
            if self.game_status == STATUS_RED_WIN:
                self.text_to_draw = "RED WIN"
            elif self.game_status == STATUS_BLUE_WIN:
                self.text_to_draw = "BLUE WIN"
            elif self.game_status >= STATUS_ROBOT1_WIN:
                self.text_to_draw = "ROBOT" + str(self.game_status - STATUS_ROBOT1_WIN + 1) + " WIN"

            # OpenCV画像をPillow画像に変換
            cv_image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(cv_image_rgb)
            
            draw = ImageDraw.Draw(pil_image)

            # 画像の高さと幅を取得
            width, height = pil_image.size

            # フォントの設定
            font_size = int(height // 8)
            font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
            font = ImageFont.truetype(font_path, font_size)

            # テキストの描画位置を計算
            text_width, text_height = draw.textsize(self.text_to_draw, font=font)
            text_x = (width - text_width) // 2
            text_y = (height - text_height) // 2

            # 四角形内にテキストを描画
            draw.text((text_x, text_y), self.text_to_draw, font=font, fill=(255, 255, 255))

            # Pillow画像をOpenCV画像に戻す
            cv_image = np.array(pil_image)
            image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
        return image

    def draw_countdown(self, image):
        if self.game_time < 0:
            draw_time = 0
        else:
            draw_time = self.game_time
            
        self.text_to_draw = str(int(draw_time // 60)).zfill(2) + " : " + str(int(draw_time % 60)).zfill(2)

        # OpenCV画像をPillow画像に変換
        cv_image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(cv_image_rgb)
        
        draw = ImageDraw.Draw(pil_image)

        # 画像の高さと幅を取得
        width, height = pil_image.size

        # フォントの設定
        font_size = int(height // 20)
        font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        font = ImageFont.truetype(font_path, font_size)

        # テキストの描画位置を計算
        text_width, text_height = draw.textsize(self.text_to_draw, font=font)
        text_x = (width - text_width) // 2
        text_y = (height - text_height) // 20

        # 四角形内にテキストを描画
        draw.text((text_x, text_y), self.text_to_draw, font=font, fill=(255, 255, 255))

        # Pillow画像をOpenCV画像に戻す
        cv_image = np.array(pil_image)
        image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        return image

    def draw_robot_status(self, image, robot_name, robot_index, position_x, position_y):
        if self.max_robot_hp[robot_index - 1] == 0:
            return image
            
        if self.robot_hp[robot_index - 1] == 0:
            background_color = (32, 32, 32)
        else:
            background_color = (128, 128, 128)

        # OpenCV画像をPillow画像に変換
        cv_image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(cv_image_rgb)

        # 画像の高さと幅を取得
        width, height = pil_image.size

        # 四角形の描画位置とサイズを計算
        rect_width = int(width / 6)  # 画像幅の1/6
        rect_height = int(height / 10)  # 画像高さの1/10
        top_left_corner = (position_x, position_y)
        bottom_right_corner = (top_left_corner[0] + rect_width, top_left_corner[1] + rect_height)

        # 四角形とテキストを描画
        draw = ImageDraw.Draw(pil_image)
        draw.rectangle([top_left_corner, bottom_right_corner], fill=background_color)

        # フォントの設定
        font_size = int(rect_height * 0.2)
        font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        font = ImageFont.truetype(font_path, font_size)

        # テキストの描画位置を計算
        text_width, text_height = draw.textsize(robot_name, font=font)
        text_x = top_left_corner[0] + (rect_width - text_width) // 2
        text_y = top_left_corner[1] + int(rect_height * 0.2)

        # 四角形内にテキストを描画
        draw.text((text_x, text_y), robot_name, font=font, fill=(255, 255, 255))

        top_left_corner = (position_x + int(rect_width * 0.1), position_y + int(rect_height * 0.6))
        bottom_right_corner = (top_left_corner[0] + int(rect_width * 0.8), top_left_corner[1] + int(rect_height*0.2))
        draw.rectangle([top_left_corner, bottom_right_corner], fill=(0,0,0))

        top_left_corner = (position_x + int(rect_width * 0.1), position_y + int(rect_height * 0.6))
        bottom_right_corner = (top_left_corner[0] + int(rect_width * 0.8 * self.robot_hp[robot_index - 1] / self.max_robot_hp[robot_index - 1]), top_left_corner[1] + int(rect_height*0.2))
        draw.rectangle([top_left_corner, bottom_right_corner], fill=(0,255,128))

        # Pillow画像をOpenCV画像に戻す
        cv_image = np.array(pil_image)
        image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        return image

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

