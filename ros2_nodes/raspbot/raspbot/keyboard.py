import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
import smbus
import time
import math
from inputs import get_key

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'keyboard', 10)
        timer_period = 0.1 # seconds between scans
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_data(self):
        event = get_key()
        for e in event:
            print(e.ev_type, e.code, e.state)
        if (event.state):
            match event.code:
                case 'KEY_W':   # forward
                    data = [100, 100]
                case 'KEY_A':   # left
                    data = [-100, 100]
                case 'KEY_S':   # back
                    data = [-100, -100]
                case 'KEY_D':   # right
                    data = [100, -100]
                case _:     # default
                    data = [0, 0]
        else:
            data = [0, 0]
        return data

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = self.get_data()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = KeyboardPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()