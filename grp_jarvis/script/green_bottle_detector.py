import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class GreenBottleDetectorNode(Node):
    def __init__(self):
        super().__init__('green_bottle_detector_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',  # Update with your camera topic
            self.image_callback,
            10
        )
        self.subscription
        self.publisher = self.create_publisher(Image, 'green_bottle/image_processed', 10)
        self.bridge = CvBridge()

        # Create a window to display the camera stream
        cv2.namedWindow('Camera Stream')

    def image_callback(self, msg):
        # Convert the ROS Image message to a NumPy array
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display the original camera stream
        cv2.imshow('Camera Stream', cv_image)
        cv2.waitKey(1)  # Allow the window to update

        # Perform color filtering
        green_filtered_image = self.filter_green(cv_image)

        # Process the filtered image (contour detection, bounding boxes, etc.)
        self.process_image(green_filtered_image)

    def filter_green(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])  # Adjust these values based on your specific green shade
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        result = cv2.bitwise_and(image, image, mask=mask)
        return result

    def process_image(self, image):
        # Add your contour detection and bounding box drawing logic here
        contours, _ = cv2.findContours(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the processed image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

        # Publish the processed image
        self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    green_bottle_detector_node = GreenBottleDetectorNode()
    rclpy.spin(green_bottle_detector_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
