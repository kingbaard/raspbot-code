import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
import smbus
import time
import math
import getch

MOTOR_POWER = 40

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'motor_control', 10)
        timer_period = 0.1 # seconds between scans
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_data(self):
        key = getch.getch()
        key = key.lower()
        match key:
            case 'w':   # forward
                print("You pressed w!")
                data = [MOTOR_POWER, MOTOR_POWER]
            case 'a':   # left
                print("You pressed a!")
                data = [-MOTOR_POWER, MOTOR_POWER]
            case 's':   # back
                print("You pressed s!")
                data = [-MOTOR_POWER, -MOTOR_POWER]
            case 'd':   # right
                print("You pressed d!")
                data = [MOTOR_POWER, -MOTOR_POWER]
            case _:     # default
                print("You pressed something else!")
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
