import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import math
import struct
import array

rosNode = None
obstacles = []

def scan_callback(scanMsg):
    global rosNode, obstacles

    header = Header()
    header.stamp = rosNode.get_clock().now().to_msg()
    header.frame_id = 'base_link'  # Change the frame_id according to your robot's configuration

    points = []
    angle = scanMsg.angle_min

    for aDistance in scanMsg.ranges:
        if 0.1 < aDistance < 5.0:
            x = math.cos(angle) * aDistance
            y = math.sin(angle) * aDistance
            z = 0.0  # Assuming a 2D laser scan, adjust if using a 3D scanner

            points.append([x, y, z])
            obstacles.append([x, y])

        angle += scanMsg.angle_increment

    # Filter points within a 45-degree range directly in front of the robot
    filtered_points = []
    for point in points:
        angle_to_point = math.atan2(point[1], point[0])
        if -math.pi / 4.0 <= angle_to_point <= math.pi / 4.0:
            filtered_points.append(point)

    # Create a PointCloud2 message for the filtered points
    filtered_point_cloud_msg = PointCloud2()
    filtered_point_cloud_msg.header = header
    filtered_point_cloud_msg.height = 1
    filtered_point_cloud_msg.width = len(filtered_points)
    filtered_point_cloud_msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    filtered_point_cloud_msg.is_bigendian = False
    filtered_point_cloud_msg.point_step = 12
    filtered_point_cloud_msg.row_step = filtered_point_cloud_msg.point_step * len(filtered_points)
    filtered_point_cloud_msg.is_dense = True

    # Convert filtered points to binary data
    data = array.array('f', [coord for point in filtered_points for coord in point])
    filtered_point_cloud_msg.data = data.tobytes()

    # Publish the filtered PointCloud2 message
    filtered_point_cloud_publisher.publish(filtered_point_cloud_msg)

    # Log obstacle points
    rosNode.get_logger().info(f"Obstacle Points:\n{obstacles}")

if __name__ == '__main__':
    rclpy.init()
    rosNode = Node('scan_interpreter')
    rosNode.create_subscription(LaserScan, 'scan', scan_callback, 10)
    filtered_point_cloud_publisher = rosNode.create_publisher(PointCloud2, 'filtered_obstacle_point_cloud', 10)

    try:
        rclpy.spin(rosNode)
    except KeyboardInterrupt:
        pass

    rosNode.destroy_node()
    rclpy.shutdown()

