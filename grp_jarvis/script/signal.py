#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool  # Import Bool message type

class ScanInterpreterNode(Node):
    def __init__(self):
        super().__init__('scan_interpreter')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.abort_publisher = self.create_publisher(Bool, 'abort_signal', 10)
    
    def scan_callback(self, scan_msg):
        # Extract the first range value from the laser scan message
        distance = scan_msg.ranges[0]

        # Check if the distance is smaller than 10 cm
        if distance < 0.1:
            self.get_logger().warn(f"Distance: {distance:.2f} m. Aborting!")
            
            # Publish an "abort signal"
            abort_msg = Bool()
            abort_msg.data = True
            self.abort_publisher.publish(abort_msg)
        else:
            self.get_logger().info(f"Distance: {distance:.2f} m. Continuing...")

def main():
    rclpy.init()
    node = ScanInterpreterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
