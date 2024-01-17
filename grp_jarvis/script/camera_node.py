#!/usr/bin/env python3

from sklearn import pipeline
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

    def capture_and_publish(self):
        # Your existing camera setup code here
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # Convert the frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

        # Publish the image message
        self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin_once(camera_node)
    camera_node.capture_and_publish()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
