from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO

# IR Sensor Pin Numbers
RIGHT1 = 11
RIGHT2 = 7
LEFT1 = 13
LEFT2 = 15


class IrPublisher(Node):
    def __init__(self):
        super().__init__('ir')
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(True)
        GPIO.setup(RIGHT1, GPIO.IN)
        GPIO.setup(RIGHT2, GPIO.IN)
        GPIO.setup(LEFT1, GPIO.IN)
        GPIO.setup(LEFT2, GPIO.IN)

        timer_period = 0.5
        self.publisher = self.create_publisher(Bool, 'ir', 10)
        self.timer = self.create_timer(timer_period, self.ir_callback)

    # If at least three sensors publish True, else false
    def ir_callback(self):
        # Set up IR sensors
        msg = Bool()
        return_count = 0
        for sensor in [RIGHT1, RIGHT2, LEFT1, LEFT2]:
            if GPIO.input(sensor):
                return_count += 1

        if return_count >= 3:
            print("ir output = False")
            msg = Bool(data=False)
        else:
            print("ir output = True")
            msg = Bool(data=True)

        self.publisher.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  
  publisher = IrPublisher()
  
  try:
    rclpy.spin(publisher)
  except Exception as e:
    print(e)
  
  GPIO.cleanup()
  publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
