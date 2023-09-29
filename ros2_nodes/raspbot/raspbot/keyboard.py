import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
import smbus
import time
import math
import getch

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'keyboard', 10)
        timer_period = 0.1 # seconds between scans
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_data(self):
        key = getch.getche()
        match key:
            case 'w':   # forward
                data = [100, 100]
            case 'a':   # left
                data = [-100, 100]
            case 's':   # back
                data = [-100, -100]
            case 'd':   # right
                data = [100, -100]
            case _:     # default
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