import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Replace with your actual topic name
            self.laser_scan_callback,
            10  # Adjust the queue size as needed
        )
        self.obstacles= []

    def control_callback(self, msg):
        rect= []
        for point in self.obstacles :
            x, y = point  # Assuming point is a tuple (x, y)

            # Check if x is in the range of -0.5 to 0.5 and y is in the range of 0 to 0.2
            if -0.5 <= x <= 0.5 and 0 <= y <= 0.2:
                rect.append(point)
        pass

    def laser_scan_callback(self, msg):
        # Specify the desired angle (in degrees)
        desired_angle = 45  # Adjust as needed

        # Convert the desired angle to index in the ranges array
        desired_index = int(desired_angle / msg.angle_increment)

        # Check if the index is within valid range
        if 0 <= desired_index < len(msg.ranges):
            desired_distance = msg.ranges[desired_index]
            self.get_logger().info(f'Distance at {desired_angle} degrees: {desired_distance} meters')

        self.obstacles= []
        angle= msg.angle_min
        for aDistance in msg.ranges :
            if 0.1 < aDistance and aDistance < 5.0 :
                aPoint= [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance
                ]
                self.obstacles.append(aPoint)
        angle+= msg.angle_increment

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
