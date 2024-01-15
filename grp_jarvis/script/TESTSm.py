import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import random

class ScanInterpreter(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.obstacles = []

        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_timer(0.2, self.control_callback)

    def scan_callback(self, scanMsg):
        #self.get_logger().info(f"scan:\n{scanMsg}")

        self.obstacles = []
        angle = scanMsg.angle_min
        for aDistance in scanMsg.ranges:
            if 0.1 < aDistance and aDistance < 10.0:
                aPoint = [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance
                ]
                self.obstacles.append(aPoint)
            angle += scanMsg.angle_increment

    def control_callback(self):
        rect = []
        x_min, x_max,y_min, y_max = -1, 1, 0.0, 0.5  # Define the x and y ranges here

        

        for point in self.obstacles:
            x, y = point


            if  y_max > y > y_min and x_min < x < x_max:
                rect.append(point)
            


        for p in rect :
            self.get_logger().info(f"Filtered obstacles: {p}")

       # return rect
        
class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        
        # Create a publisher for sending Twist messages to control the robot's movement
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a subscriber to receive an abort signal
        self.abort_subscriber = self.create_subscription(Bool, '/abort_signal', self.abort_callback, 10)
        
        # Flag to track whether an abort signal has been received
        self.abort_signal_received = False

    # Method to move the robot randomly or turn based on the abort signal
    def move_randomly(self):

        rect=ScanInterpreter.control_callback

        

        while rclpy.ok():
            self.get_logger().info("Running...")

            

            # If no abort signal is received, move randomly
            if not self.abort_signal_received:
                # Generate random linear and angular velocities
                velo = Twist()
                velo.linear.x = 0.5 # Random linear velocity between -0.5 and 0.5 m/s
                
                self.velocity_publisher.publish(velo)
            else:

                if rect[len-1[0]]< 0.2:
                
                    self.abort_signal_received=True


                # If abort signal is received, stop the robot and turn until the signal is not received
                self.get_logger().info("Abort signal received. Turning...")
                self.velocity_publisher.publish(Twist())  # Stop the robot

                # Turn until the abort signal is not received
                while self.abort_signal_received and rclpy.ok():
                    self.get_logger().info("Turning...")
                    # Set angular velocity to turn the robot in place
                    self.velocity_publisher.publish(Twist(angular=Twist.Angular(z=0.5)))
                    rclpy.spin_once(self, timeout_sec=0.1)

            # Clear the abort signal flag if it's no longer received
            if not self.abort_signal_received:
                self.get_logger().info("Abort signal cleared.")

            rclpy.spin_once(self, timeout_sec=0.1)

    # Callback function for handling the abort signal
    def abort_callback(self, msg):
        self.abort_signal_received = msg.data
        

def main():
    rclpy.init()

    scan_interpreter = ScanInterpreter()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(scan_interpreter)
    
    node=MoveNode()
    node.move_randomly()

    node.destroy_node()
    rclpy.shutdown()

    print("tuto_move :: STOP.")

    try:
        executor.spin()
    finally:
        executor.shutdown()
        scan_interpreter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()