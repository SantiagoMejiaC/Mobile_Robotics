#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import random



# Define a ROS node class for controlling the robot's movement
class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        
        # Create a publisher for sending Twist messages to control the robot's movement
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a subscriber to receive an abort signal
        self.stopSub = self.create_subscription(Bool, 'Stop', self.ScanInterpreter.control_callback, 10)
        
        # Flag to track whether an abort signal has been received
        self.abort_signal_received = True

    # Method to move the robot randomly or turn based on the abort signal
    def move_randomly(self):


        while rclpy.ok():
            self.get_logger().info("Running...")

            

            # If no abort signal is received, move randomly
            if  self.abort_signal_received == False:
                # Generate random linear and angular velocities
                velo = Twist()
                velo.linear.x = 0.5 # Random linear velocity between -0.5 and 0.5 m/s
                
                self.velocity_publisher.publish(velo)
            else:
                # If abort signal is received, stop the robot and turn until the signal is not received
                self.get_logger().info("Abort signal received. Turning...")
                self.velocity_publisher.publish(Twist(0))  # Stop the robot

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

# Main function to initialize the ROS node and run the robot movement control
def main():
    rclpy.init()
    node = MoveNode()
    node.move_randomly()

    node.destroy_node()
    rclpy.shutdown()

    print("tuto_move :: STOP.")

# Entry point to start the program
if __name__ == '__main__':
    main()

