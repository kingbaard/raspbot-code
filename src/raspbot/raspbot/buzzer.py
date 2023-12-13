from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Char, Bool, Int32
from sensor_msgs.msg import Range

import smbus
import RPi.GPIO as GPIO
import time

# IR Sensor Pin Numbers
BUZZER_PIN = 32

# Notes
class Notes(Enum):
    C_LOW = 248
    D_LOW = 278
    E_LOW = 294
    F_LOW = 330
    G_LOW = 371
    A_LOW = 416
    B_LOW = 467

    C = 495
    D = 556
    E = 624
    F = 661
    G = 742
    A = 833
    B = 935

    C_HIGH = 990
    D_HIGH = 1112
    E_HIGH = 1178
    F_HIGH = 1322
    G_HIGH = 1484
    A_HIGH = 1665
    B_HIGH = 1869

class Buzzer(Node):
    def __init__(self):
        super().__init__('buzzer')

        # Set up IR sensors
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(BUZZER_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(BUZZER_PIN, Notes.C.value)
        self.play_init_sound()

        self.subscription = self.create_subscription(Int32, '/delivery_state', self.state_callback, 10)
        self.state_hist = [0, 0]

    def state_callback(self, msg):
        self.state_hist[0] = self.state_hist[1]
        self.state_hist[1] = msg.data
        print(self.state_hist)

        #Found box condition
        match self.state_hist:
            case [0, 1]:
                print("Playing found box tune")
                self.play_found_box()
            case [3, 4]:
                self.play_delivery_success()
            case _:
                pass

    def play_init_sound(self):
        music = [
            [Notes.C_HIGH, 0.75]
            ]
        self.play_song(music)
        time.sleep(0.25)
        music = [
            [Notes.E_HIGH, 0.75]
            ]
        self.play_song(music)
        time.sleep(0.25)
        music = [
            [Notes.C_HIGH, 0.75]
            ]
        self.play_song(music)

    def play_found_box(self):
        music = [
            [Notes.C, 1],
            [Notes.D, 0.5],
            [Notes.E, 0.5],
            [Notes.G, 1.5],
        ]
        self.play_song(music)

    # If at least three sensors publish True, else false
    def play_delivery_success(self):
        pass

    def play_song(self, notes):
        self.pwm.start(100)
        for note in notes:
            self.pwm.ChangeFrequency(note[0].value)
            time.sleep(note[1])
        self.pwm.stop()
        


def main(args=None):
  rclpy.init(args=args)
  
  subscriber = Buzzer()
  
  try:
    rclpy.spin(subscriber)
  except Exception as e:
    print(e)
  
  subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()