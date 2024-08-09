import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

GAME_INDIVIDUAL_MATICH = 0
GAME_TEAM_MATCH        = 1

STATUS_NORMAL =     0
STATUS_RED_WIN =    1
STATUS_BLUE_WIN =   2
STATUS_ROBOT1_WIN = 3
STATUS_ROBOT2_WIN = 4
STATUS_ROBOT3_WIN = 5
STATUS_ROBOT4_WIN = 6

class GameManager(Node):

    def __init__(self):
        super().__init__('GameManager')
        self.declare_parameter('initial_time', 90)
        self.initial_time = self.get_parameter('initial_time').get_parameter_value().integer_value
        self.declare_parameter('game_kind', 0)
        self.game_kind = self.get_parameter('game_kind').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Int32, 'countdown', 10)
        self.status_publisher_ = self.create_publisher(Int32, 'game_status', 10)

        self.remaining_time = -3
        self.game_status = STATUS_NORMAL

        self.publish_timer = self.create_timer(0.2, self.timer_callback)
        self.countdown_timer = self.create_timer(1.0, self.progress_callback)

        self.robot_hp = [0,-1,-1,-1,-1,-1,-1,-1]
        self.subscription_1 = self.create_subscription(Int32, 'robot1_hp', self.robot1_hp_callback, 10)
        self.subscription_2 = self.create_subscription(Int32, 'robot2_hp', self.robot2_hp_callback, 10)
        self.subscription_3 = self.create_subscription(Int32, 'robot3_hp', self.robot3_hp_callback, 10)
        self.subscription_4 = self.create_subscription(Int32, 'robot4_hp', self.robot4_hp_callback, 10)
        self.subscription_5 = self.create_subscription(Int32, 'robot5_hp', self.robot5_hp_callback, 10)
        self.subscription_6 = self.create_subscription(Int32, 'robot6_hp', self.robot6_hp_callback, 10)
        self.subscription_7 = self.create_subscription(Int32, 'robot7_hp', self.robot7_hp_callback, 10)
        self.subscription_8 = self.create_subscription(Int32, 'robot8_hp', self.robot8_hp_callback, 10)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.remaining_time
        self.publisher_.publish(msg)

        status_msg = Int32()
        status_msg.data = self.game_status
        self.status_publisher_.publish(status_msg)

    def progress_callback(self):
        if self.remaining_time == -1: # 試合開始処理
            self.remaining_time = self.initial_time
        elif self.remaining_time < 0: # 試合開始前カウントダウン
            self.remaining_time += 1
        elif self.remaining_time > 0: # 試合中カウントダウン
            self.remaining_time -= 1
        else: # 勝敗決定
            if self.game_kind == GAME_INDIVIDUAL_MATICH:
                # カウントダウン終了時に最大HPを持つロボットの番号を計算してパブリッシュ
                self.game_status = self.robot_hp.index(max(self.robot_hp)) + STATUS_ROBOT1_WIN
                result_msg = Int32()
                result_msg.data = self.game_status

    def robot1_hp_callback(self, msg):
        if self.remaining_time > 0:
            self.robot_hp[0] = msg.data

    def robot2_hp_callback(self, msg):
        if self.remaining_time > 0:
            self.robot_hp[1] = msg.data

    def robot3_hp_callback(self, msg):
        if self.remaining_time > 0:
            self.robot_hp[2] = msg.data

    def robot4_hp_callback(self, msg):
        if self.remaining_time > 0:
            self.robot_hp[3] = msg.data

    def robot5_hp_callback(self, msg):
        if self.remaining_time > 0:
            self.robot_hp[4] = msg.data

    def robot6_hp_callback(self, msg):
        if self.remaining_time > 0:
            self.robot_hp[5] = msg.data

    def robot7_hp_callback(self, msg):
        if self.remaining_time > 0:
            self.robot_hp[6] = msg.data

    def robot8_hp_callback(self, msg):
        if self.remaining_time > 0:
            self.robot_hp[7] = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

