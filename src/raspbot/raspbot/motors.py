from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Char, Bool, Int32MultiArray
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

MOTOR_POWER = 75
MOTOR_OFFSET = 25
APRIL_TAG_MIDDLE = 275
APRIL_TAG_OFFSET = 30
SLEEP_TIME = .5

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
    self.keyboard_subscription = self.create_subscription(Char, '/keyboard_control', self.keyboard_callback, 10)
    self.warehouse_subscription = self.create_subscription(Bool, '/warehouse_control', self.warehouse_callback, 10)
    self.april_tag_subscription = self.create_subscription(Int32MultiArray, '/apriltags', self.april_tag_callback, 10)
    self.sonar_subscription = self.create_subscription(Range, '/sonar', self.sonar_callback, 10)

    # IMU Publisher
    # timer_period = 0.5 # seconds between publish
    # self.motor_publisher = self.create_publisher(Int32MultiArray, 'imu_control', 10)
    # self.current_control = [0, 0]
    # self.timer = self.create_timer(timer_period, self.publish_control)

    # Initial values
    # self.servo1_angle = -1
    # self.servo2_angle = -1
    self.e_stop = False
    self.state = States.SEARCH
    self.sonar_distance = None
    self.box_id = None
    self.box_x_pos = None
    self.target_box_id = None
    self.target_box_x_pos = None
    self.goal_id = None
    self.goal_x_pos = None
    self.target_goal_id = None
    self.target_goal_x_pos = None
    self.completed = []
  
  # def motor_callback(self, msg):
  #   self.current_control = [msg.data[0], msg.data[1]]

  #   motor_msg = Int32MultiArray()
  #   motor_msg.data = [msg.data[0], msg.data[1]]
  #   print(msg.data)
  #   self.car.control_car(msg.data[0], msg.data[1])
  
  # def servo_callback(self, msg):
  #   if msg.data[0] != self.servo1_angle:
  #     self.car.set_servo(1, msg.data[0])
  #     self.servo1_angle = msg.data[0]
  #   if msg.data[1] != self.servo2_angle:
  #     self.car.set_servo(2, msg.data[1])
  #     self.servo2_angle = msg.data[1]

  def keyboard_callback(self, msg):
    print(f"You pressed '{chr(msg.data)}'")
    match chr(msg.data):
      case 'w':   # forward
          self.car.control_car(MOTOR_POWER, MOTOR_POWER)
      case 'a':   # left
          self.car.control_car(-MOTOR_POWER, MOTOR_POWER)
      case 's':   # back
          self.car.control_car(-MOTOR_POWER, -MOTOR_POWER)
      case 'd':   # right
          self.car.control_car(MOTOR_POWER, -MOTOR_POWER)
      case 'e':   # Toggle E-stop
          self.car.control_car(0, 0)
          self.e_stop = not self.e_stop
          print(f"E-STOP = {'ON' if self.e_stop else 'OFF'}")
      case '0':   # State 0
          self.state = States.SEARCH
      case '1':   # State 1
          self.state = States.ACQUIRE
      case '2':   # State 2
          self.state = States.FIND_GOAL
      case '3':   # State 3
          self.state = States.DELIVER
      case '4':   # State 4
          self.state = States.RESET
      case _:   # default
          self.car.control_car(0, 0)        

  # def publish_control(self):
  #   motor_msg = Int32MultiArray()
  #   motor_msg.data = self.current_control
  #   print(self.current_control)
  #   self.motor_publisher.publish(motor_msg)

  # def drive_square_callback(self, msg):
  #   motor_msg = Int32MultiArray()
  #   if msg.data:
  #     for _ in range(4):
  #       # Drive forward
  #       motor_msg.data = [50, 50]
  #       self.motor_publisher.publish(motor_msg)
  #       self.car.control_car(50, 50)
  #       time.sleep(3)

  #       # Turn left
  #       motor_msg.data = [-100, 50]
  #       self.motor_publisher.publish(motor_msg)
  #       self.car.control_car(-100, 50)
  #       time.sleep(2)

  #   motor_msg.data = [0, 0]
  #   self.motor_publisher.publish(motor_msg)
  #   self.car.control_car(0,0)
  
  def warehouse_callback(self, msg):
    if msg.data and not self.e_stop:
      # Pause for a second and reset april tag
      # time.sleep(SLEEP_TIME)
      # self.car.control_car(0, 0) 
      # self.box_id = None
      # self.box_x_pos = None
      # self.goal_id = None
      # self.goal_x_pos = None
      # time.sleep(SLEEP_TIME)

      

      match (self.state):
        case States.SEARCH:
          print("State: SEARCH")
          if self.target_box_id is not None:
            self.car.control_car(0, 0)
            self.state = States.ACQUIRE
          else:
            self.car.control_car(-(MOTOR_POWER - MOTOR_OFFSET), MOTOR_POWER)

        case States.ACQUIRE:
          print("State: ACQUIRE")
          print(f"{self.sonar_distance}")
          if self.sonar_distance < .06: 
            # Box acquired
            self.car.control_car(0, 0)
            self.state = States.FIND_GOAL
          else:
            if self.target_box_x_pos > APRIL_TAG_MIDDLE + APRIL_TAG_OFFSET:
              # Slight turn right
              self.car.control_car(50, -50)
            elif self.target_box_x_pos < APRIL_TAG_MIDDLE - APRIL_TAG_OFFSET:
              # Slight turn left
              self.car.control_car(-50, 50)
            else:
              # Box centered
              self.car.control_car(MOTOR_POWER, MOTOR_POWER)

        case States.FIND_GOAL:
          print("State: FIND_GOAL")
          print(f"target goal {self.target_goal_id} target box {self.target_box_id}")
          if self.target_goal_id is not None and self.target_goal_id == self.target_box_id + 3:
            # Found correct goal
            self.car.control_car(0, 0)
            self.state = States.DELIVER
          else:
            self.car.control_car(-(MOTOR_POWER - MOTOR_OFFSET), MOTOR_POWER) 

        case States.DELIVER:
          print("State: DELIVER")
          if not self.goal_id:
            # Arrived at goal (can't see goal april tag anymore)
            self.completed.append(self.target_box_id)
            self.car.control_car(0, 0)
            self.state = States.RESET
          else:
            if self.target_goal_x_pos > APRIL_TAG_MIDDLE + APRIL_TAG_OFFSET:
              # Slight turn right
              self.car.control_car(50, -50)
            elif self.target_goal_x_pos < APRIL_TAG_MIDDLE - APRIL_TAG_OFFSET:
              # Slight turn left
              self.car.control_car(-50, 50)
            else:
              # Goal centered
              self.car.control_car(MOTOR_POWER, MOTOR_POWER)

        case States.RESET:
          print("State: RESET")
          if self.sonar_distance >= 1:
            # Backed up now reset state and go again
            self.car.control_car(0, 0)
            self.state = States.SEARCH
            self.box_id = None
            self.box_x_pos = None
            self.target_box_id = None
            self.target_box_x_pos = None
            self.goal_id = None
            self.goal_x_pos = None
            self.target_goal_id = None
            self.target_goal_x_pos = None
          else:
            self.car.control_car(-MOTOR_POWER, -MOTOR_POWER)

        case _:
          print("ERROR")
          self.e_stop = True
          self.car.control_car(0, 0)
    else:
      self.car.control_car(0, 0)

  def april_tag_callback(self, msg):
    # msg.data = [id, x_pos]
    print(f"{msg.data}")
    if msg.data[0] < 3:
      print(f"FOUND BOX {msg.data[0]} AT {msg.data[1]}")
      self.box_id = msg.data[0]
      self.box_x_pos = msg.data[1]
      if (self.target_box_id is None or self.box_id == self.target_box_id) and self.box_id not in self.completed:
         self.target_box_id = self.box_id
         self.target_box_x_pos = self.box_x_pos
         self.target_goal_id = self.target_box_id + 3
    else:
      print(f"FOUND GOAL {msg.data[0]} AT {msg.data[1]}")
      self.goal_id = msg.data[0]
      self.goal_x_pos = msg.data[1]
      if self.goal_id == self.target_goal_id:
        self.target_goal_x_pos = self.goal_x_pos
  
  def sonar_callback(self, msg):
    if msg.range < 0:
      self.sonar_distance = np.inf
    else:
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
