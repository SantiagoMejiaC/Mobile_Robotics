#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node 

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
import random
#from kobuki_ros_interfaces.msg import WheelDropEvent

class ScanInterpreter(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        
        # self.obstacles = []
        # self.left_obs=[]
        # self.right_obs=[]
        # self.leftWheelDropped = False
        # self.rightWheelDropped = False
        # self.stopped = True
        self.cmd_vel_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        #cmd_vel if is simulation and /multi/cmd_nav'if is tbot
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        #self.create_subscription(WheelDropEvent, 'events/wheel_drop', self.wheel_callback, 50)
        # self.create_timer(0.05,self.control_callback,10)
        self.cmd_vel_msg = Twist()


    # def wheel_callback(self, wheel_msg):
    #     rightWheel = wheel_msg.wheel==1
    #     Dropped = wheel_msg.state==1

    #     if rightWheel and Dropped:
    #         self.rightWheelDropped = True

    #     elif rightWheel and not Dropped:
    #         self.rightWheelDropped = False

    #     elif not rightWheel and Dropped:
    #         self.leftWheelDropped = True

    #     elif not rightWheel and not Dropped:
    #         self.leftWheelDropped = False

    #     if self.robotLift():
    #         self.stopped = True
    #         self.cmd_vel_msg.angular.z = 0.0  # Adjust the angular velocity as needed
    #         self.cmd_vel_msg.linear.x =0.0
    #         print('lift')

    #     elif self.robotGround() and self.stopped:
    #         time.sleep(2)
    #         self.stopped = False
    #         print('groun')
        

    def scan_callback(self, scan_msg):
        self.obstacles = []

       
        angle = scan_msg.angle_min
        for a_distance in scan_msg.ranges:

            if 0.1 < a_distance and a_distance < 1:
                a_point = [
                    math.cos(angle) * a_distance,
                    math.sin(angle) * a_distance,
                    0.0
                ]
                self.obstacles.append(a_point)
        
                


            angle += scan_msg.angle_increment

        #if self.stopped == False:
        self.control_callback()
        
    

  


    # def robotLift(self):
    #      return self.leftWheelDropped and self.rightWheelDropped
    
    # def robotGround(self):
    #      return  not self.leftWheelDropped and not self.rightWheelDropped


    def isTurning(self):
        return self.cmd_vel_msg.angular.z != 0.0
    
    def isObstacleFound(self):
        return (len(self.left_obs)+ len(self.right_obs)) != 0.0

    def control_callback(self):
        self.left_obs=[]
        self.right_obs=[]
        if self.isTurning() and self.isObstacleFound() :
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            return
        
        self.cmd_vel_msg = Twist()

        
        for a_point in self.obstacles:
            
            if 0.1<a_point[0]<0.5 and 0.01 < a_point[1] < 0.2 :
                    self.left_obs.append(a_point)

            if 0.1<a_point[0]<0.5 and -0.01 > a_point[1] > -0.2 :
                    self.right_obs.append(a_point)     


        if len(self.left_obs)>0 or len(self.left_obs)>len(self.right_obs):
                self.cmd_vel_msg.angular.z = -1.5  # Adjust the angular velocity as needed
                self.cmd_vel_msg.linear.x =0.0
        
        elif len(self.right_obs)> len(self.left_obs):
                self.cmd_vel_msg.angular.z = 1.5  # Adjust the angular velocity as needed
                self.cmd_vel_msg.linear.x =0.0
        else:
            self.cmd_vel_msg.linear.x = 0.3# Forward linear velocity when no obstacles
            self.cmd_vel_msg.angular.z=0.0

        self.cmd_vel_publisher.publish(self.cmd_vel_msg)


          #  if y_max > y and y > y_min and x_min < x and x < x_max:
           #     rect.append(point)
          
        

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
