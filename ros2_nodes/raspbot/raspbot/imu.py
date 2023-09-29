import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Bool
from geometry.msg import PoseStamped

from numpy import sin, cos
import matplotlib.pyplot as plt
import smbus
import time
import math

# Car speed 
# Power | Speed m/s
# 50    | .16
# 100   | .42
# 200   | .765
# 1     | .0042 ish?
# The linear function connecting (50, 0.16) and (100, 0.42) is y=0.0052x -0.1

# Angular Velocity
# lPower    | rPower    | Angular v rad/s
# -100      | 100       | 0.785398

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu')
        #Create log
        self.x = 0
        self.y = 0
        self.heading = 0
        self.hist = []
        self.motor_subscription = self.create_subscription(Int32MultiArray, '/motor_control', self.motor_callback, 10)
        self.position_publisher = self.create_publisher(PoseStamped, '/position', 10)
        self.seq = 1

    def motor_callback(self, msg):
        current_time = time.time()
        if self.last_time:
            delta_time = current_time - self.last_time

        #Find distance traveled if not turning
        if msg[0] == msg[1]:
            velocity = 0.0052 * msg[0] - 0.1
            x_d = velocity * sin(self.heading) * delta_time
            y_d = velocity * cos(self.heading) *  delta_time

            self.x += x_d
            self.y += y_d
        
        #Find change in heading if turning
        # Currently only accounts for turns where one motor is set to 100 and the other to -100
        if abs(msg[0] - msg[1] > 10):
            angular_velocity = 0
            if msg[0] == -100 and msg[1] == 100:
                angular_velocity = -0.785398
            elif msg[0] == 100 and msg[1] == -100:
                angular_velocity = 0.785398
            self.heading += angular_velocity * delta_time
        
        self.last_time = current_time

    def publish_pose(self):
        pose_msg = PoseStamped()
        
        pose_msg.header.seq = self.seq
        self.seq += 1
        pose_msg.header.stamp = time.time()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0

        pose_msg.pose.orientation.z = sin(self.heading/2)
        pose_msg.pose.orientation.w = cos(self.heading/2)

        self.position_publisher.publish(pose_msg)


            

def main(args=None):
    rclpy.init(args=args)
  
    imuPublisher = ImuPublisher()

    try:
        rclpy.spin(imuPublisher)
    except Exception as e:
        print(e)

    imuPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()