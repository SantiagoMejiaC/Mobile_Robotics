#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
import random
from kobuki_ros_interfaces.msg import WheelDropEvent

class ScanInterpreter(Node):

    def __init__(self):
        super().__init__('scan_interpreter')

        #Variables initialization  for the obstacle detection and the emergency stop when lifted
        self.obstacles = []
        self.left_obs=[]
        self.right_obs=[]
        self.leftWheelDropped = False
        self.rightWheelDropped = False
        self.stopped = False
        self.cmd_vel_msg = Twist()

        
        #Creation of the publishers and subcriptions for the topics
        #publisher for the'/multi/cmd_nav' topic who is in charge of publishing the velocity to the multiplexer
        self.cmd_vel_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
               
        #Surcriptions from the'scan' and 'events/wheel_drop' topic who are in charge of recieving info from the        
        #wheels event and the leader Scan
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(WheelDropEvent, 'events/wheel_drop', self.wheel_callback, 50)
        



     def robotLift(self): #Function that returns if the robot is lifted 
         return self.leftWheelDropped and self.rightWheelDropped
    
    def robotGround(self): #Function that returns if the robot is Dropped 
         return  not self.leftWheelDropped and not self.rightWheelDropped

    def wheel_callback(self, wheel_msg): #Function in charge of detecting if the wheels are lifted
        rightWheel = wheel_msg.wheel==1  #The robot has a layout where the right wheel is 1 and left wheel is 0
        Dropped = wheel_msg.state==1     #and gives a response if the wheel is Lifted of 1 and 0 if its in the ground

        if rightWheel and Dropped:
            self.rightWheelDropped = True

        elif rightWheel and not Dropped:
            self.rightWheelDropped = False

        elif not rightWheel and Dropped:
            self.leftWheelDropped = True

        elif not rightWheel and not Dropped:
            self.leftWheelDropped = False

        if self.robotLift(): 
            self.stopped = True
            self.cmd_vel_msg.angular.z = 0.0  # Adjust the angular velocity as needed
            self.cmd_vel_msg.linear.x =0.0    # Adjust the linear velocity as needed
            print('lift')

        elif self.robotGround() and self.stopped:
            time.sleep(2)
            self.stopped = False
            print('groun')


    

   


    def scan_callback(self, scan_msg): #Function in charge of detecting possible obstacles for the autonomous moving
        self.obstacles = []

       
        angle = scan_msg.angle_min
        for a_distance in scan_msg.ranges:

            if 0.1 < a_distance and a_distance < 1:
                a_point =                            
                    math.cos(angle) * a_distance,
                    math.sin(angle) * a_distance,
                    0.0
                ]
                self.obstacles.append(a_point) #Create a list of Points (obstacles) so it can be sorted 
        
                


            angle += scan_msg.angle_increment



        
        if self.stopped == False:       #It checks if the robot is lifted or in the ground to call the 
           self.control_callback()      #control_callback function to create the move
            
        
    

  

    def isTurning(self):                             #Function that returns if the robot is turning  
        return self.cmd_vel_msg.angular.z != 0.0
    
    def isObstacleFound(self):                                       #Function that returns if the robot found an object by
        return (len(self.left_obs)+ len(self.right_obs)) != 0.0      #seeing if the left and right obstacle list is empty  


    def control_callback(self):     #Function in charge of creating and publishing possible the velocities for the autonomous moving
        self.left_obs=[]
        self.right_obs=[]
        if self.isTurning() and self.isObstacleFound() :        #detects if the robot is still turning and if found an object to not create false obstacles
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            return
        
        self.cmd_vel_msg = Twist() #resets the velocity variable

        
        for a_point in self.obstacles:      #From the obstacles list it sorts if the object is in the right or left from the robot 
            
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
