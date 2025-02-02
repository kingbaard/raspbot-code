from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Char, Bool, Int32MultiArray, Int32
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

MOTOR_POWER = 50
APRIL_TAG_MIDDLE = 275
APRIL_TAG_OFFSET = 100

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
    self.ir_subscription = self.create_subscription(Bool, '/ir', self.ir_callback, 10)

    # State Publisher
    timer_period = 0.5 # seconds between publish
    self.state_publisher = self.create_publisher(Int32, 'delivery_state', 10)
    self.state_publish_timer = self.create_timer(timer_period, self.publish_state)

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
    self.old_time = time.time()
    self.tag_memory = {}
    self.action_clock = 0
    self.is_driving = False
    self.is_dark_floor = False
    self.last_turned_left = False
    self.last_sonars = [4 for x in range(10)]
    self.last_irs = [False for x in range(5)]
  
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
  
  def warehouse_callback(self, msg):
        if msg.data and not self.e_stop:
            # Handle Clock Management
            elapsed = time.time() - self.old_time
            self.old_time = time.time()
            self.update_tag_memory(elapsed)
            self.update_is_driving(elapsed)
            acceptable_delta = APRIL_TAG_OFFSET/self.sonar_distance
            print('acceptable_delta:',acceptable_delta)
            if not self.is_driving:
                self.car.control_car(0,0)
            match (self.state):
                case States.SEARCH:
                    print("State: SEARCH")
                    if self.target_box_id is not None:
                        self.car.control_car(0, 0)
                        self.state = States.ACQUIRE
                    else:
                        if self.is_driving:
                            self.car.control_car(-MOTOR_POWER, MOTOR_POWER)

                case States.ACQUIRE:
                    print("State: ACQUIRE")
                    print(f"{self.sonar_distance}")
                    if self.tag_memory[self.target_box_id]['valid'] == 0:
                        if self.last_turned_left:
                            self.target_box_x_pos = 500
                        else:
                            self.target_box_x_pos = 0
                        self.tag_memory[self.target_box_id]['valid'] = 1
                    if self.sonar_distance < .08: 
                        # Box acquired
                        self.car.control_car(0, 0)
                        self.state = States.FIND_GOAL
                    else:
                        if self.is_driving:
                            if self.target_box_x_pos > APRIL_TAG_MIDDLE + acceptable_delta:
                            # Slight turn right
                                self.car.control_car(MOTOR_POWER, -MOTOR_POWER)
                                self.last_turned_left = False
                            elif self.target_box_x_pos < APRIL_TAG_MIDDLE - acceptable_delta:
                            # Slight turn left
                                self.car.control_car(-MOTOR_POWER, MOTOR_POWER)
                                self.last_turned_left = True
                            else:
                            # Box centered
                                self.car.control_car(MOTOR_POWER, MOTOR_POWER)

                case States.FIND_GOAL:
                    print("State: FIND_GOAL")
                    print(f"target goal {self.target_goal_id} target box {self.target_box_id}")
                    if self.target_goal_id in self.tag_memory and self.tag_memory[self.target_goal_id]['valid'] > 0:
                        # Found correct goal
                        self.car.control_car(0, 0)
                        self.state = States.DELIVER
                        self.delivery_start = True
                    else:
                        if self.is_driving:
                            self.car.control_car(-MOTOR_POWER, MOTOR_POWER) 

                case States.DELIVER:
                    print("State: DELIVER")
                    if self.is_dark_floor:
                          self.completed.append(self.target_box_id)
                          self.car.control_car(0, 0)
                          self.state = States.RESET
                    if self.tag_memory[self.target_goal_id]['valid'] <= 0:
                        if self.delivery_start:
                            # Uh oh, we lost the goal, going back to find goal
                            self.state = States.FIND_GOAL
                            return
                    else:
                        # Arrived at goal (or location with similar floor material)
                        if self.is_dark_floor:
                          self.completed.append(self.target_box_id)
                          self.car.control_car(0, 0)
                          self.state = States.RESET
                        if self.is_driving:
                            if self.target_goal_x_pos > APRIL_TAG_MIDDLE + acceptable_delta:
                                # Slight turn right
                                self.car.control_car(MOTOR_POWER, -MOTOR_POWER)
                            elif self.target_goal_x_pos < APRIL_TAG_MIDDLE - acceptable_delta:
                                # Slight turn left
                                self.car.control_car(-MOTOR_POWER, MOTOR_POWER)
                            else:
                                # Goal centered
                                self.delivery_start = False
                                self.car.control_car(MOTOR_POWER, MOTOR_POWER)

                case States.RESET:
                    print("State: RESET")
                    if self.sonar_distance >= 2:
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
                        if self.is_driving:
                            self.car.control_car(-MOTOR_POWER, -MOTOR_POWER)

                case _:
                    print("ERROR")
                    self.e_stop = True
                    self.car.control_car(0, 0)
        else:
            self.car.control_car(0, 0)

  def update_is_driving(self, elapsed_time):
     self.action_clock -= elapsed_time
     if self.action_clock < 0:
        if self.is_driving:
            self.is_driving = False
            self.action_clock = 0.5
        else:
            self.is_driving = True
            if self.state in (States.ACQUIRE, States.DELIVER):
                self.action_clock = 0.25
            else:
               self.action_clock = 0.5
            
  def update_tag_memory(self, elapsed_time):
     for tag in self.tag_memory.values():
        tag['valid'] -= elapsed_time
        if tag['valid'] < 0:
           tag['valid'] = 0

  def april_tag_callback(self, msg):
    # msg.data = [id, x_pos]
    print(f"{msg.data}")
    if msg.data[0] < 3:
      print(f"FOUND BOX {msg.data[0]} AT {msg.data[1]}")
      self.tag_memory[msg.data[0]] = {"pos": msg.data[1], "valid":1.5}
      self.box_id = msg.data[0]
      self.box_x_pos = msg.data[1]
      if (self.target_box_id is None or self.box_id == self.target_box_id) and self.box_id not in self.completed:
         self.target_box_id = self.box_id
         self.target_box_x_pos = self.box_x_pos
         self.target_goal_id = self.target_box_id + 3
    else:
      print(f"FOUND GOAL {msg.data[0]} AT {msg.data[1]}")
      self.tag_memory[msg.data[0]] = {"pos": msg.data[1], "valid":3}
      self.goal_id = msg.data[0]
      self.goal_x_pos = msg.data[1]
      if self.goal_id == self.target_goal_id:
        self.target_goal_x_pos = self.goal_x_pos
  
  def sonar_callback(self, msg):
      if msg.range < 0:
          range = 4
      else:
          range = msg.range
      self.last_sonars.append(range)
      self.last_sonars = self.last_sonars[1:]
      self.sonar_distance = np.average(self.last_sonars)

  def ir_callback(self, msg):
      ir_result = True if msg.data else False

      self.last_irs.append(ir_result)
      self.last_irs = self.last_irs[1:]
      self.is_dark_floor = sum(self.last_irs) > 4

  def publish_state(self):
      msg = Int32()
      msg.data = self.state.value
      print(f"self.state.value: {type(self.state.value)}")
      self.state_publisher.publish(msg)


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
