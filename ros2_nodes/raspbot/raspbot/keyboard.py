import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
import smbus
import time
import math
from pynput import keyboard

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'keyboard', 10)
        timer_period = 0.1 # seconds between scans
        self.timer = self.create_timer(timer_period, self.timer_callback)

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

    def on_press(self, key):
        print('key {0} pressed'.format(key.char))
        match key.char:
            case 'w':   # forward
                self.data = [100, 100]
            case 'a':   # left
                self.data = [-100, 100]
            case 's':   # back
                self.data = [-100, -100]
            case 'd':   # right
                self.data = [100, -100]
            case _:     # default
                self.data = [0, 0]

    def on_release(self, key):
        self.data = [0, 0]

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = self.data
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = KeyboardPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()