#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import pyrealsense2 as rs
import math
import rclpy
from std_msgs.msg import Int8MultiArray, String
import time

# RealSense setup
pipeline = rs.pipeline()
config = rs.config()
colorizer = rs.colorizer()

# FPS set to 30
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

align_to = rs.stream.depth
align = rs.align(align_to)

# Template matching setup
img_rgb_template = cv.imread('car.png')
img_gray_template = cv.cvtColor(img_rgb_template, cv.COLOR_BGR2GRAY)
template = cv.imread('template.png', 0)
w, h = template.shape[::-1]
threshold_template = 0.3

# Segmentation parameters using HSV color space
color = 55

lo = np.array([color - 15, 40, 40])
hi = np.array([color + 6, 255, 255])

# Flag to check if the template is detected
object_detected = False

# Distance threshold for object detection (increased distance)
distance_threshold = 10.0  # Adjust as needed

# Creating morphological kernel
kernel = np.ones((3, 3), np.uint8)

# ROS 2 initialization
rclpy.init()
node = rclpy.create_node('distance_pub_node')

# Use the Point message type for the publisher
publisher = node.create_publisher(Int8MultiArray, 'obj_coord', 10)
Bootledetect = node.create_publisher(String, 'NodeDetector', 10)

# Time interval for publishing (in seconds)
publish_interval = 0.5
last_publish_time = time.time()

try:
    while rclpy.ok():
        # RealSense frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not aligned_color_frame:
            continue

        # Colorized depth map
        colorized_depth = colorizer.colorize(depth_frame)
        depth_colormap = np.asanyarray(colorized_depth.get_data())

        # Set background to black
        depth_colormap[depth_colormap == 0] = 0

        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
        color_image = np.asanyarray(aligned_color_frame.get_data())

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # Use pixel value of depth-aligned color image to get 3D axes
        x, y = int(color_colormap_dim[1] / 2), int(color_colormap_dim[0] / 2)
        depth = depth_frame.get_distance(x, y)
        dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x, y], depth)
        distance = math.sqrt(((dx) ** 2) + ((dy) ** 2) + ((dz) ** 2))
        # Create a list of coordinates [x, y, z]
        coords = [dx, dy, dz]

        # Create a message of type Int8MultiArray
        coords_msg = Int8MultiArray()
        coords_msg.data = [int(round(value, 0)) for value in coords]

        # Publish the message
        publisher.publish(coords_msg)

        # Check if the object is within the specified distance
        if distance < distance_threshold:
            # Convert color image to HSV
            hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

            # Segmentation in HSV color space
            mask = cv.inRange(hsv_image, lo, hi)
            mask = cv.erode(mask, kernel, iterations=1)
            mask = cv.dilate(mask, kernel, iterations=1)
            image_segmented = cv.bitwise_and(color_image, color_image, mask=mask)

            # Template matching with segmented image
            img_gray_segmented = cv.cvtColor(image_segmented, cv.COLOR_BGR2GRAY)
            res = cv.matchTemplate(img_gray_segmented, template, cv.TM_CCOEFF_NORMED)
            loc = np.where(res >= threshold_template)

            # Reset the object_detected flag
            object_detected = False
            distance_text = ""

            for pt in zip(*loc[::-1]):
                rect_center = ((pt[0] + pt[0] + w) // 2, (pt[1] + pt[1] + h) // 2)
                cv.rectangle(color_image, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)

                # Display distance in the center of the rectangle
                distance_text = str(round(distance, 2))
                text_size = cv.getTextSize(distance_text, cv.FONT_HERSHEY_DUPLEX, 1, 1)[0]
                text_position = (rect_center[0] - text_size[0] // 2, rect_center[1] + text_size[1] // 2)
                cv.putText(color_image, distance_text, text_position, cv.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1,
                           cv.LINE_AA)

                # Draw a circle at the center of the rectangle
                cv.circle(color_image, rect_center, 10, (0, 0, 255), 2)

                object_detected = True

            # Show images
            cv.imshow('RealSense', image_segmented)
            cv.waitKey(1)

            # Print object detection status in real-time
            if object_detected:
                msg = String()
                msg.data = "Object detected"
                Bootledetect.publish(msg)
            else:
                msg2 = String()
                msg2.data = "Object not detected"
                Bootledetect.publish(msg2)

except Exception as e:
    print(e)

finally:
    pipeline.stop()
    node.destroy_node()
    rclpy.shutdown()
