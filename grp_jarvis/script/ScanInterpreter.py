import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import Bool

class ScanInterpreter(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.obstacles = []
        self.obstacle_msg = Bool()

        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.stop_publisher = self.create_publisher(Bool, '/move', 10)

        # Add a subscription for control signals
        self.create_subscription(Bool, '/control', self.control_callback, 10)

    def scan_callback(self, scan_msg):
        self.obstacles = []
        angle = scan_msg.angle_min
        for a_distance in scan_msg.ranges:
            if 0.1 < a_distance and a_distance < 10.0:
                a_point = [
                    math.cos(angle) * a_distance,
                    math.sin(angle) * a_distance
                ]
                self.obstacles.append(a_point)
            angle += scan_msg.angle_increment

    def control_callback(self, bool_msg):
        rect = []
        x_min, x_max, y_min, y_max = -1, 1, 0.0, 0.5

        for point in self.obstacles:
            x, y = point

            obstacle = False

            if y_max > y > y_min and x_min < x < x_max:
                rect.append(point)
                obstacle = True

        for p in rect:
            self.get_logger().info(f"Filtered obstacles: {p}")

        # Use the received bool_msg to publish the stop signal
        self.obstacle_msg.data = obstacle
        self.stop_publisher.publish(self.obstacle_msg)

def main():
    rclpy.init()

    scan_interpreter = ScanInterpreter()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(scan_interpreter)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        scan_interpreter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
