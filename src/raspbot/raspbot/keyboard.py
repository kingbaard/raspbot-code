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
        self.publisher = self.create_publisher(Int32MultiArray, 'keyboard_control', 10)
        timer_period = 0.1 # seconds between scans
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_data(self):
        key = getch.getch()
        key = key.lower()
        return [key]

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
