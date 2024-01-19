#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.camera_info_subscription = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.object_publisher = self.create_publisher(Image, '/object_detection', 10)

        # Initialize RealSense pipeline and configure streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

        # Initialize object detection parameters (similar to the first code)
        self.colorGreen = 50
        self.lo = np.array([self.colorGreen-5, 100, 50])
        self.hi = np.array([self.colorGreen+5, 255, 255])

    def camera_info_callback(self, msg):
        # Use camera info for any necessary camera calibration or rectification
        pass

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Apply color mask for object detection
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, self.lo, self.hi)
            result_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Draw rectangles around detected objects
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Convert the processed image back to ROS format
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.object_publisher.publish(processed_image_msg)

            # Display the image
            cv2.imshow('Object Detection', cv_image)
            cv2.waitKey(1)  # Adjust the delay as needed

        except Exception as e:
            self.get_logger().error(str(e))

def main():
    rclpy.init()
    detection_node = ObjectDetectionNode()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
