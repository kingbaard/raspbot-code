from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Char, Bool, Int32MultiArray
from sensor_msgs.msg import Range

import smbus
import RPi.GPIO as GPIO

# IR Sensor Pin Numbers
RIGHT1 = 11
RIGHT2 = 7
LEFT1 = 13
LEFT2 = 15


class IrPublisher(Node):
    def __init__(self):
        super().__init__('ir')

        # Set up IR sensors
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(RIGHT1, GPIO.IN)
        GPIO.setup(RIGHT2, GPIO.IN)
        GPIO.setup(LEFT1, GPIO.IN)
        GPIO.setup(LEFT1, GPIO.IN)

        timer_period = 0.5
        self.publisher = self.create_publisher(Bool, 'ir', 10)
        self.timer = self.create_timer(timer_period, self.ir_callback)

    # If at least three sensors publish True, else false
    def ir_callback(self):
        return_count = 0
        for sensor in [RIGHT1, RIGHT2, LEFT1, LEFT2]:
            if GPIO.input(sensor):
                return_count += 1

        if return_count >= 3:
            print("ir output = True")
            self.publisher.publish(True)
        else:
            print("ir output = False")
            self.publisher.publish(False)

    # IMU Publisher
    # timer_period = 0.5 # seconds between publish
    # self.motor_publisher = self.create_publisher(Int32MultiArray, 'imu_control', 10)
    # self.current_control = [0, 0]
    # self.timer = self.create_timer(timer_period, self.publish_control)


def main(args=None):
  rclpy.init(args=args)
  
  subscriber = IrPublisher()
  
  try:
    rclpy.spin(subscriber)
  except Exception as e:
    print(e)
    subscriber.car.stop()
  
  subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
