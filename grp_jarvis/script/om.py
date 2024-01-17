#!/usr/bin/python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import random

class ScanInterpreter(Node):

    def __init__(self):
        super().__init__('scan_interpreter')

        self.obstacles = []
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 20)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 20)

        # Initialize variables for turning
        self.turning = False
        self.turn_start_time = 0.0
        self.turn_duration = 0.0

    def scan_callback(self, scan_msg):
        self.obstacles = []
        left_obs = []
        right_obs = []
        angle = scan_msg.angle_min

        for a_distance in scan_msg.ranges:
            if 0.1 < a_distance and a_distance < 10.0:
                a_point = [
                    math.cos(angle) * a_distance,
                    math.sin(angle) * a_distance,
                    0.0
                ]
                self.obstacles.append(a_point)

                if 0.1 < a_point[0] < 0.5 and 0.01 < a_point[1] < 0.3:
                    left_obs.append(a_point)

                if 0.1 < a_point[0] < 0.5 and -0.01 > a_point[1] > -0.3:
                    right_obs.append(a_point)

            angle += scan_msg.angle_increment

        cmd_vel_msg = Twist()

        if len(left_obs) > 0 or len(right_obs) > 0:
            if  self.turning == False:
                # Start turning when obstacles are detected
                self.turn_start_time = self.get_clock().now().nanoseconds / 1e9  # Current time in seconds
                self.turn_duration = random.uniform(500, 1000)  # Random turn duration between 1 and 5 seconds
                self.turning = True

            # Check if it's time to stop turning
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed_time = current_time - self.turn_start_time
            if elapsed_time < self.turn_duration:
                # Continue turning
                cmd_vel_msg.angular.z = 0.5  # Adjust the angular velocity as needed
            else:
                # Stop turning
                self.turning = False
                self.turn_start_time = 0.0
                self.turn_duration = 0.0
        else:
            # Move forward when no obstacles
            cmd_vel_msg.linear.x = 0.3  # Forward linear velocity

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
