import cv2
import numpy as np
import pyrealsense2 as rs
import signal

# Connect to the RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
pipeline.start(config)

# Capture ctrl-c event
isOk = True
def signalInterruption(signum, frame):
    global isOk
    print("\nCtrl-c pressed")
    isOk = False

signal.signal(signal.SIGINT, signalInterruption)

image_count = 191 # Initialize image_count to 0

while isOk:
    # Wait for a coherent tuple of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    if not color_frame:
        continue

    # Convert color image to numpy array
    color_image = np.asanyarray(color_frame.get_data())

    # Display the color image
    cv2.imshow("RealSense Color Image", color_image)

    # Press 'c' to capture an image and select ROI
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        # Select ROI
        r = cv2.selectROI(color_image)
        cv2.destroyAllWindows()

        # Crop image
        imCrop = color_image[int(r[1]):int(r[1] + r[3]), int(r[0]):int(r[0] + r[2])]

        # Calculate average color values
        average_h = np.mean(imCrop[:, :, 0])
        average_s = np.mean(imCrop[:, :, 1])
        average_v = np.mean(imCrop[:, :, 2])

        print("Average H: {}, Average S: {}, Average V: {}".format(average_h, average_s, average_v))

        # Display cropped image
        cv2.imshow("Cropped Image", imCrop)

        # Save the cropped image
        image_count += 1
        filename = "cropped_image_{}.jpg".format(image_count)
        cv2.imwrite(filename, imCrop)
        print("Cropped Image saved as:", filename)

# Stop streaming
print("\nEnding...")
pipeline.stop()
cv2.destroyAllWindows()
