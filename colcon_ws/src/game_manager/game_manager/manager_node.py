import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import Joy
import struct
from multiprocessing import shared_memory

GAME_INDIVIDUAL_MATICH = 0
GAME_TEAM_MATCH        = 1

STATUS_DRAW =       -1
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

        try:
            self.shm = shared_memory.SharedMemory(name='isaac_sim_reset', size=struct.calcsize('i'), create=False)
        except FileNotFoundError:
            self.shm = shared_memory.SharedMemory(name='isaac_sim_reset', size=struct.calcsize('i'), create=True)
            struct.pack_into('i', self.shm.buf, 0, 0)

        self.is_reset = False
        self.remaining_time = 0
        self.show_result_time = 0
        self.game_status = STATUS_NORMAL

        self.publish_timer = self.create_timer(0.2, self.timer_callback)
        self.countdown_timer = self.create_timer(1.0, self.progress_callback)

        self.publisher = self.create_publisher(Bool, '/reset_hp', 10)
        self.robot_hp = [0,-1,-1,-1,-1,-1,-1,-1]
        self.subscription_1 = self.create_subscription(Int32, 'robot1_hp', self.robot1_hp_callback, 10)
        self.subscription_2 = self.create_subscription(Int32, 'robot2_hp', self.robot2_hp_callback, 10)
        self.subscription_3 = self.create_subscription(Int32, 'robot3_hp', self.robot3_hp_callback, 10)
        self.subscription_4 = self.create_subscription(Int32, 'robot4_hp', self.robot4_hp_callback, 10)
        self.subscription_5 = self.create_subscription(Int32, 'robot5_hp', self.robot5_hp_callback, 10)
        self.subscription_6 = self.create_subscription(Int32, 'robot6_hp', self.robot6_hp_callback, 10)
        self.subscription_7 = self.create_subscription(Int32, 'robot7_hp', self.robot7_hp_callback, 10)
        self.subscription_8 = self.create_subscription(Int32, 'robot8_hp', self.robot8_hp_callback, 10)

        self.joy_status = [0,0,0,0,0,0,0,0]
        self.joy_subscription_1 = self.create_subscription(Joy, '/sample_robot_1/joy', self.joy1_callback, 10)
        self.joy_subscription_2 = self.create_subscription(Joy, '/sample_robot_2/joy', self.joy2_callback, 10)
        self.joy_subscription_3 = self.create_subscription(Joy, '/sample_robot_3/joy', self.joy3_callback, 10)
        self.joy_subscription_4 = self.create_subscription(Joy, '/sample_robot_4/joy', self.joy4_callback, 10)
        self.joy_subscription_5 = self.create_subscription(Joy, '/sample_robot_5/joy', self.joy5_callback, 10)
        self.joy_subscription_6 = self.create_subscription(Joy, '/sample_robot_6/joy', self.joy6_callback, 10)
        self.joy_subscription_7 = self.create_subscription(Joy, '/sample_robot_7/joy', self.joy7_callback, 10)
        self.joy_subscription_8 = self.create_subscription(Joy, '/sample_robot_8/joy', self.joy8_callback, 10)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.remaining_time
        self.publisher_.publish(msg)

        status_msg = Int32()
        status_msg.data = self.game_status
        self.status_publisher_.publish(status_msg)

    def progress_callback(self):
        self.game_status = STATUS_NORMAL

        shm_value = struct.unpack_from('i', self.shm.buf, 0)[0]
        if self.is_reset == False:
            for i in range(8):
                if self.joy_status[i] == 1:
                    self.is_reset = True
                    break
            if self.is_reset == True:
                struct.pack_into('i', self.shm.buf, 0, shm_value + 1)
                print(f"Reset: {shm_value + 1}")
                self.remaining_time = -10
                self.show_result_time = 10

                self.publisher.publish(Bool(data=True))
        else:
            is_status_all_clear = True
            for i in range(8):
                if self.joy_status[i] == 1:
                    is_status_all_clear = False
                    break
            if is_status_all_clear == True:
                self.is_reset = False

        if self.remaining_time == -1: # 試合開始処理
            self.remaining_time = self.initial_time
        elif self.remaining_time < 0: # 試合開始前カウントダウン
            self.remaining_time += 1
        elif self.remaining_time > 0: # 試合中カウントダウン
            self.remaining_time -= 1
        elif self.show_result_time >= 0: # 勝敗決定
            self.show_result_time -= 1
            if self.game_kind == GAME_INDIVIDUAL_MATICH:
                # 最大HPを持つロボットが複数いる場合は引き分け
                if self.robot_hp.count(max(self.robot_hp)) > 1:
                    self.game_status = STATUS_DRAW
                else:
                    # カウントダウン終了時に最大HPを持つロボットの番号を計算してパブリッシュ
                    self.game_status = self.robot_hp.index(max(self.robot_hp)) + STATUS_ROBOT1_WIN

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

    def joy1_callback(self, msg):
        # ボタン8と9が同時に押されたらリセット
        if msg.buttons[8] == True and msg.buttons[9] == True:
            self.joy_status[0] = 1
        else:
            self.joy_status[0] = 0

    def joy2_callback(self, msg):
        # ボタン8と9が同時に押されたらリセット
        if msg.buttons[8] == True and msg.buttons[9] == True:
            self.joy_status[1] = 1
        else:
            self.joy_status[1] = 0

    def joy3_callback(self, msg):
        # ボタン8と9が同時に押されたらリセット
        if msg.buttons[8] == True and msg.buttons[9] == True:
            self.joy_status[2] = 1
        else:
            self.joy_status[2] = 0

    def joy4_callback(self, msg):
        # ボタン8と9が同時に押されたらリセット
        if msg.buttons[8] == True and msg.buttons[9] == True:
            self.joy_status[3] = 1
        else:
            self.joy_status[3] = 0

    def joy5_callback(self, msg):
        # ボタン8と9が同時に押されたらリセット
        if msg.buttons[8] == True and msg.buttons[9] == True:
            self.joy_status[4] = 1
        else:
            self.joy_status[4] = 0

    def joy6_callback(self, msg):
        # ボタン8と9が同時に押されたらリセット
        if msg.buttons[8] == True and msg.buttons[9] == True:
            self.joy_status[5] = 1
        else:
            self.joy_status[5] = 0

    def joy7_callback(self, msg):
        # ボタン8と9が同時に押されたらリセット
        if msg.buttons[8] == True and msg.buttons[9] == True:
            self.joy_status[6] = 1
        else:
            self.joy_status[6] = 0

    def joy8_callback(self, msg):
        # ボタン8と9が同時に押されたらリセット
        if msg.buttons[8] == True and msg.buttons[9] == True:
            self.joy_status[7] = 1
        else:
            self.joy_status[7] = 0

def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

