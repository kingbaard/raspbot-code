import msvcrt
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
import smbus
import time
import math

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'keyboard', 10)

    def keyboard_callback(self):
        msg = Int32MultiArray()
        while True:
            key = ord(msvcrt.getwch())
            if key == 27: #ESC
                break
            elif key == 224: #Special keys (arrows, f keys, ins, del, etc.)
                key = ord(msvcrt.getwch())
                if key == 72: #Up arrow
                    msg[0] = 100
                    msg[1] = 100
                elif key == 80: #Down arrow
                    msg[0] = -100
                    msg[1] = -100
                elif key == 75: #Left arrow
                    msg[0] = -100
                    msg[1] = 100
                elif key == 77: #Right arrow
                    msg[0] = 100
                    msg[1] = -100
                self.publisher.publish(msg)

    def main(args=None):
        rclpy.init(args=args)
        publisher = KeyboardPublisher()
        rclpy.spin(publisher)
        publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()