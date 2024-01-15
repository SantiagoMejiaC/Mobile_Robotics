#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

import cv2 as cv
import numpy as np
import pyrealsense2 as rs
import signal
import time
import sys

# Function to handle Ctrl-c interruption
def signal_interruption(signum, frame):
    global is_ok
    print("\nCtrl-c pressed")
    is_ok = False

# Configure RealSense depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)

# Start RealSense streaming
pipeline.start(config)

# Configure template matching
img_rgb = cv.imread('green.png')
img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
template = cv.imread('mask.png', 0)
w, h = template.shape[::-1]
threshold = 0.8

# Capture Ctrl-c event
is_ok = True
signal.signal(signal.SIGINT, signal_interruption)

# Main loop
count = 1
ref_time = time.process_time()
freq = 60

sys.stdout.write("-")

while is_ok:
    # RealSense frames
    frames = pipeline.wait_for_frames()
    color_frame = frames.first(rs.stream.color)
    depth_frame = frames.first(rs.stream.depth)

    if not (depth_frame and color_frame):
        continue

    # Convert RealSense images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth image
    depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    sys.stdout.write(f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)")

    # Template matching
    res = cv.matchTemplate(img_gray, template, cv.TM_CCOEFF_NORMED)
    loc = np.where(res >= threshold)

    # Draw bounding boxes for matched objects
    for pt in zip(*loc[::-1]):
        cv.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)

    # Display images
    images = np.hstack((color_image, depth_colormap))
    cv.namedWindow('RealSense', cv.WINDOW_AUTOSIZE)
    cv.imshow('RealSense', images)
    cv.waitKey(1)

    # Frequency calculation
    if count == 10:
        new_time = time.process_time()
        freq = 10 / ((new_time - ref_time))
        ref_time = new_time
        count = 0
    count += 1

# Stop streaming
print("\nEnding...")
pipeline.stop()
