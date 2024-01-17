#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node 

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
import random

class ScanInterpreter(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        
        self.obstacles = []
        self.cmd_vel_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        #cmd_vel if is simulation and /multi/cmd_nav'if is tbot
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        # self.create_timer(0.05,self.control_callback,10)
       
        

    def scan_callback(self, scan_msg):
        self.obstacles = []
       
        angle = scan_msg.angle_min
        for a_distance in scan_msg.ranges:

            if 0.1 < a_distance and a_distance < 0.8:
                a_point = [
                    math.cos(angle) * a_distance,
                    math.sin(angle) * a_distance,
                    0.0
                ]
                self.obstacles.append(a_point)

                


            angle += scan_msg.angle_increment
        self.control_callback()
        

#      if len(left_obs)>0 or len(left_obs)>len(right_obs):
#                cmd_vel_msg.angular.z = -1.5  # Adjust the angular velocity as needed
#        
#        elif len(right_obs)> len(left_obs):
#                cmd_vel_msg.angular.z = 1.5  # Adjust the angular velocity as needed
#        else:
#            cmd_vel_msg.linear.x = 0.3  # Forward linear velocity when no obstacles
#        
#         
#        self.cmd_vel_publisher.publish(cmd_vel_msg)


    def isTurning(self):
        return self.cmd_vel_msg.angular.z != 0
    
    def isObstacleFound(self):
        return len(self.obstacles) != 0

    def control_callback(self):
        if self.isTurning() and self.isObstacleFound():
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            return
        
        left_obs=[]
        right_obs=[]
        
        self.cmd_vel_msg = Twist()
        for a_point in self.obstacles:
            
            if 0.1<a_point[0]<0.5 and 0.01 < a_point[1] < 0.2 :
                    left_obs.append(a_point)

            if 0.1<a_point[0]<0.5 and -0.01 > a_point[1] > -0.2 :
                    right_obs.append(a_point)     


        if len(left_obs)>0 or len(left_obs)>len(right_obs):
                cmd_vel_msg.angular.z = -1.5  # Adjust the angular velocity as needed
        
        elif len(right_obs)> len(left_obs):
                cmd_vel_msg.angular.z = 1.5  # Adjust the angular velocity as needed
        else:
            cmd_vel_msg.linear.x = 0.3  # Forward linear velocity when no obstacles
        
         
        self.cmd_vel_publisher.publish(cmd_vel_msg)


          #  if y_max > y and y > y_min and x_min < x and x < x_max:
           #     rect.append(point)



#
#        if rect :  # If obstacles are present
#
#            
#            obstacle = True
#            left_count = sum(1 for x,y in rect if y < 0)
#            right_count = sum(1 for x,y in rect if y > 0)
#
#            if left_count > right_count:
#                self.turn_left = True
#                self.turn_right = False
#            else:
#                self.turn_left = False
#                self.turn_right = True
#
#        else:
#            obstacle = False
#            self.turn_left = False
#            self.turn_right = False
#
#        # Publish velocity command based on obstacle distribution
#        
#        if  obstacle:
#
#            #cmd_vel_msg.linear.x = 0
#
            
        

def main():
    rclpy.init()

    scan_interpreter = ScanInterpreter()
    
    while True:
        rclpy.spin_once(scan_interpreter,timeout_sec=0.1 )
   
    scan_interpreter.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
