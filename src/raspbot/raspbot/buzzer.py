from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Char, Bool, Int32MultiArray
from sensor_msgs.msg import Range

import smbus
import RPi.GPIO as GPIO
import time

# IR Sensor Pin Numbers
BUZZER_PIN = 32

# Notes
class Notes(Enum):
    C = 248
    D = 278
    E = 294
    F = 330
    G = 371
    A = 416
    B = 467

    B1 = 495
    B2 = 556
    B3 = 624
    B4 = 661
    B5 = 742
    B6 = 833
    B7 = 935

class Buzzer(Node):
  def __init__(self):
    super().__init__('ir')

    # Set up IR sensors
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)

    self.pwm = GPIO.PWM(BUZZER_PIN, Notes.C)

    timer_period = 0.5
    self.subscriber = self.subscriber

    def play_found_box(self):
       music = [
          [Notes.C, 1],
          [Notes.D, 0.5],
          [Notes.E, 0.5],
          [Notes.G, 1.5],
       ]
       play_song(music)

    # If at least three sensors publish True, else false
    def play_delivery_success(self):
        pass

    def play_song(self, notes):
        self.pwm.start(50)
        for note in notes:
           self.pwm.ChangeFrequency(note[0])
           time.sleep(note[1])
        self.pwm.stop()
        


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