from enum import Enum
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Bool
from sensor_msgs.msg import Range

import smbus
import time
import math

# Car speed 
# Power | Speed m/s
# 50    | .16
# 100   | .42
# 200   | .765
# 1     | .0042 ish?

class Car:
    def __init__(self):
        self._addr = 0x16
        self._device = smbus.SMBus(1)

    def __write_u8(self, register, data):
        try:
            self._device.write_byte_data(self._addr, register, data)
        except:
            print('write_u8 error')

    def __write_register(self, register):
        try:
            self._device.write_byte(self._addr, register)
        except:
            print('write_register error')

    def __write_array(self, register, data):
        try:
            self._device.write_i2c_block_data(self._addr, register, data)
        except:
            print('write_array error')

    def control_car(self, left, right):
        """
        left: int (-255, 255)
        right: int (-255, 255)

        sets the motor with the speed given (not actually in unit, just a power amount)
        """
        register = 0x01
        left_direction = 0 if left < 0 else 1
        right_direction = 0 if right < 0 else 1

        if left < 0:
            left *= -1
        if right < 0:
            right *= -1

        data = [left_direction, left, right_direction, right]
        self.__write_array(register, data)

    def stop(self):
        register = 0x02
        self.__write_u8(register, 0x00)

    def set_servo(self, servo_id, angle):
        register = 0x03
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        data = [servo_id, angle]
        self.__write_array(register, data)

class MinimalSubscriber(Node):
  def __init__(self):
    super().__init__('motors')
    self.car = Car()

    # Subscriptions
    # self.motor_subscription = self.create_subscription(Int32MultiArray, '/motor_control', self.motor_callback, 10)
    # self.servo_subscription = self.create_subscription(Int32MultiArray, '/servo_control', self.servo_callback, 10)
    # self.drive_square_subscription = self.create_subscription(Bool, '/drive_square_control', self.drive_square_callback, 10)
    self.keyboard_subscription = self.create_subscription(Int32MultiArray, '/keyboard_control', self.keyboard_callback, 10)
    self.warehouse_subscription = self.create_subscription(Bool, '/warehouse_control', self.warehouse_callback, 10)
    self.april_tag_subscription = self.create_subscription(Int32MultiArray, '/april_tag_control', self.april_tag_callback, 10)
    self.sonar_subscription = self.create_subscription(Range, '/sonar_control', self.sonar_callback, 10)

    # IMU Publisher
    timer_period = 0.5 # seconds between publish
    self.motor_publisher = self.create_publisher(Int32MultiArray, 'imu_control', 10)
    self.current_control = [0, 0]
    self.timer = self.create_timer(timer_period, self.publish_control)

    # Initial values
    self.servo1_angle = -1
    self.servo2_angle = -1
    self.e_stop = False
    self.state = States.SEARCH
    self.sonar_distance = None
    self.completed = []
    self.target_acquired = False
    self.package_received = False
    self.goal_found = False
    self.package_delivered = False
  
  def motor_callback(self, msg):
    self.current_control = [msg.data[0], msg.data[1]]

    motor_msg = Int32MultiArray()
    motor_msg.data = [msg.data[0], msg.data[1]]
    print(msg.data)
    self.car.control_car(msg.data[0], msg.data[1])
  
  def servo_callback(self, msg):
    if msg.data[0] != self.servo1_angle:
      self.car.set_servo(1, msg.data[0])
      self.servo1_angle = msg.data[0]
    if msg.data[1] != self.servo2_angle:
      self.car.set_servo(2, msg.data[1])
      self.servo2_angle = msg.data[1]

  def keyboard_callback(self, msg):
    self.car.control_car(msg.data[0], msg.data[1])
    if msg.data[0] == 0 and msg.data[1] == 0:
      # Toggle E-stop and reset
      print("E-STOP")
      self.e_stop = not self.e_stop
      self.state = States.RESET
      self.completed = []

  def publish_control(self):
    motor_msg = Int32MultiArray()
    motor_msg.data = self.current_control
    print(self.current_control)
    self.motor_publisher.publish(motor_msg)

  def drive_square_callback(self, msg):
    motor_msg = Int32MultiArray()
    if msg.data:
      for _ in range(4):
        # Drive forward
        motor_msg.data = [50, 50]
        self.motor_publisher.publish(motor_msg)
        self.car.control_car(50, 50)
        time.sleep(3)

        # Turn left
        motor_msg.data = [-100, 50]
        self.motor_publisher.publish(motor_msg)
        self.car.control_car(-100, 50)
        time.sleep(2)

    motor_msg.data = [0, 0]
    self.motor_publisher.publish(motor_msg)
    self.car.control_car(0,0)
  
  def warehouse_control(self, msg):
    if msg.data and not self.e_stop:
      match (self.state):
        case States.SEARCH:
          print("State: SEARCH")
          if self.target_acquired:
            self.car.control_car(0, 0)
            self.state = States.ACQUIRE
          else:
            self.car.control_car(-100, 50)

        case States.ACQUIRE:
          print("State: ACQUIRE")
          if self.sonar_distance < .2: 
            # self.package_received = True
            self.car.control_car(0, 0)
            self.state = States.FIND_GOAL
          else:
            self.car.control_car(100, 100)

        case States.FIND_GOAL:
          print("State: FIND_GOAL")
          if self.goal_found:
            self.car.control_car(0, 0)
            self.state = States.DELIVER
          else:
            self.car.control_car(-100, 50) 

        case States.DELIVER:
          print("State: DELIVER")
          if self.package_delivered:
            self.car.control_car(0, 0)
            # Add delivered box to completed boxes?
            # self.completed.append()
            self.state = States.RESET
          else:
            self.car.control_car(100, 100)

        case States.RESET:
          print("State: RESET")
          if self.sonar_distance >= 5:
            self.state = States.SEARCH
            self.target_acquired = False
            # self.package_received = False
            self.goal_found = False
            self.package_delivered = False
          else:
            self.car.control_car(-100, -100)

        case _:
          print("ERROR")
          self.car.control_car(0, 0)
    else:
      self.car.control_car(0, 0)

  def april_tag_control(self, msg):
    # Recognize april tag, check if already completed
    self.target_acquired = msg.data[0]
    self.goal_found = msg.data[1]
  
  def sonar_control(self, msg):
    self.sonar_distance = msg.range

class States(Enum):
  SEARCH = 0
  ACQUIRE = 1
  FIND_GOAL = 2
  DELIVER = 3
  RESET = 4

def main(args=None):
  rclpy.init(args=args)
  
  subscriber = MinimalSubscriber()
  
  try:
    rclpy.spin(subscriber)
  except Exception as e:
    print(e)
    subscriber.car.stop()
  
  subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
