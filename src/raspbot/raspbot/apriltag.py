import cv2
import numpy as np
from apriltag import apriltag
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32MultiArray
import cv_bridge
import cv2
import numpy as np
import os
import time
import pickle


class Apriltag(Node):
    box_ids = [0,2,4]
    dest_ids = [1,3,5]
    def __init__(self):
        super().__init__('apriltag')
        self.image_subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.apriltag_callback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, 'apriltags', 10)

    def apriltag_callback(self, image_msg):
        try:
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            print("np_arr created")
            img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE) # Try color if this doesn't work
            # print("cv_image created", img)

            detector = apriltag("tagStandard41h12")

            detections = detector.detect(img)

            for detection in detections:
                print(detection)
                tag_id = detection["tag_id"]
                tag_cx = detection["center"][0]
                payload = Int32MultiArray(data=[tag_id,tag_cx])
                self.publisher.publish(payload)


        except Exception as e:
            print('Error processing image: %s' % str(e))



def main(args=None):
    rclpy.init(args=args)

    apriltag = Apriltag()

    try:
        rclpy.spin(apriltag)
    except Exception as e:
        print(f"error:{e}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
