import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ScanInterpreter(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.obstacles = []
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.turn_left = False
        self.turn_right = False

        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
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

            if y_max > y > y_min and x_min < x < x_max:
                rect.append(point)

        if rect:  # If obstacles are present
            obstacle = True
            left_count = sum(1 for x, y in rect if x < 0)
            right_count = sum(1 for x, y in rect if x > 0)

            if left_count > right_count:
                self.turn_left = True
                self.turn_right = False
            else:
                self.turn_left = False
                self.turn_right = True

        else:
            obstacle = False
            self.turn_left = False
            self.turn_right = False

        # Publish velocity command based on obstacle distribution
        cmd_vel_msg = Twist()
        if obstacle:
            if self.turn_left:
                cmd_vel_msg.angular.z = 0.5  # Adjust the angular velocity as needed
            elif self.turn_right:
                cmd_vel_msg.angular.z = -0.5  # Adjust the angular velocity as needed

        self.cmd_vel_publisher.publish(cmd_vel_msg)

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
