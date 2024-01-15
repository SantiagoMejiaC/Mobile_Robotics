import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanInterpreter(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.obstacles = []

        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        self.stopPublisher=self.create_publisher(bool,'/move',10)
        #self.create_timer(2, self.stopPublisher)
       

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

    def control_callback(self,bool):
        rect = []
        x_min, x_max,y_min, y_max = -1, 1, 0.0, 0.5  # Define the x and y ranges here

        

        for point in self.obstacles:
            x, y = point

            obstacle=False


            if  y_max > y > y_min and x_min < x < x_max:
                rect.append(point)
                obstacle=True
                
            


        for p in rect :
            self.get_logger().info(f"Filtered obstacles: {p}")
            self.stopPublisher.publish(obstacle)

       # return rect
        
        

def main():
    rclpy.init()

    scan_interpreter = ScanInterpreter()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(scan_interpreter)

    
    executor.spin()

    executor.shutdown()
    scan_interpreter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()