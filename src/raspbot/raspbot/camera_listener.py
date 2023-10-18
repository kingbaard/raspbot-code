import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv_bridge
import cv2
import numpy as np
import os
import time
import pickle


class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener')
        self.frame_count = 0
        self.video_count = 0
        self.image_subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.recorder_callback, 10)
        self.video_writer = None
        self.image_array: np.array =  np.array()

    def recorder_callback(self, image_msg):
        try:
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            print("np_arr created")
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            print("cv_image created", cv_image)
            if self.video_writer == None:
                self.init_video_writer(cv_image)
                print("video_writer inited")
            if not self.image_array:
                pass
            self.video_writer.write(cv_image)
            print("vw.write called")
            self.frame_count += 1
            if self.frame_count == 1000:
                print("writing to file..")
                self.frame_count = 0
                self.video_writer.release()
                self.video_writer = None
                self.video_count += 1

        except Exception as e:
            print('Error processing image: %s' % str(e))

    def save_np_array(self, array):
        pass 

    def init_video_writer(self, image):
        try:
            height, width, _ = image.shape
            video_format = 'mp4'  # or any other video format supported by OpenCV
            # video_filename = str(time.time()) + video_format
            video_filename = f"gaming{self.video_count}." + video_format
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 60  # Frames per second

            # Save Video
            self.video_writer = cv2.VideoWriter(
                video_filename, fourcc, fps, (width, height))
            if not self.video_writer.isOpened():
                self.video_writer.open()
            time.sleep(3)
        except Exception as e:
            print('Error initializing video writer: %s' % str(e))

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    subscriber = CameraListener()

    try:
        rclpy.spin(subscriber)
    except Exception as e:
        print(f"error:{e}")

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
