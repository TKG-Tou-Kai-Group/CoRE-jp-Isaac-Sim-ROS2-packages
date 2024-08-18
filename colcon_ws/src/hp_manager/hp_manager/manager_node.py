import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
import threading

class HPManager(Node):

    def __init__(self):
        super().__init__('hp_manager')
        self.declare_parameter('initial_hp', 100)
        self.initial_hp = self.get_parameter('initial_hp').get_parameter_value().integer_value
        self.declare_parameter('respawn_time_sec', 30.0)
        self.respawn_time_sec = self.get_parameter('respawn_time_sec').get_parameter_value().double_value

        self.hp = self.initial_hp
        self.hp_publisher = self.create_publisher(Int32, 'robot_hp', 10)

        subscriber_1 = self.create_subscription(Bool, 'armor_topic_1', self.bool1_callback, 10)
        subscriber_2 = self.create_subscription(Bool, 'armor_topic_2', self.bool2_callback, 10)
        subscriber_3 = self.create_subscription(Bool, 'armor_topic_3', self.bool3_callback, 10)
        subscriber_4 = self.create_subscription(Bool, 'armor_topic_4', self.bool4_callback, 10)
 
        self.old_bool_values = [False, False, False, False]

        # Subscriber for reset command
        self.reset_subscriber = self.create_subscription(Bool, 'reset_hp', self.reset_callback, 10)

        self.publish_hp_timer = self.create_timer(1.0, self.publish_hp)

    def bool1_callback(self, msg):
        if msg.data and self.old_bool_values[0] == False:
            if self.hp > 0:
                self.hp -= 10
                if self.hp <= 0:
                    self.hp = 0
                    if self.respawn_time_sec > 0:
                        threading.Timer(self.respawn_time_sec, self.reset_hp).start()
        self.old_bool_values[0] = msg.data

    def bool2_callback(self, msg):
        if msg.data and self.old_bool_values[1] == False:
            if self.hp > 0:
                self.hp -= 10
                if self.hp <= 0:
                    self.hp = 0
                    if self.respawn_time_sec > 0:
                        threading.Timer(self.respawn_time_sec, self.reset_hp).start()
        self.old_bool_values[1] = msg.data

    def bool3_callback(self, msg):
        if msg.data and self.old_bool_values[2] == False:
            if self.hp > 0:
                self.hp -= 10
                if self.hp <= 0:
                    self.hp = 0
                    if self.respawn_time_sec > 0:
                        threading.Timer(self.respawn_time_sec, self.reset_hp).start()
        self.old_bool_values[2] = msg.data

    def bool4_callback(self, msg):
        if msg.data and self.old_bool_values[3] == False:
            if self.hp > 0:
                self.hp -= 10
                if self.hp <= 0:
                    self.hp = 0
                    if self.respawn_time_sec > 0:
                        threading.Timer(self.respawn_time_sec, self.reset_hp).start()
        self.old_bool_values[3] = msg.data

    def reset_hp(self):
        self.hp = self.initial_hp

    def reset_callback(self, msg):
        if msg.data:
            self.reset_hp()

    def publish_hp(self):
        hp_msg = Int32()
        hp_msg.data = self.hp
        self.hp_publisher.publish(hp_msg)

def main(args=None):
    rclpy.init(args=args)
    hp_manager = HPManager()
    rclpy.spin(hp_manager)

    hp_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
