
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        
        # Subscribe to LIDAR scans
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        
        #self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

        # Distance threshold in meters (e.g., stop if an object is closer than 0.5 meters)
        self.obstacle_distance_threshold = 0.15

    def lidar_callback(self, msg):
        # Calculate the minimum distance from the scan data
        min_distance = min([distance for distance in msg.ranges if distance > 0])

        # Check if the minimum distance is less than the obstacle threshold
        if min_distance < self.obstacle_distance_threshold:
            # Stop the robot if an obstacle is too close
            self.stop_moving()
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f} meters, stopping.')
        else:
            # Otherwise, keep moving forward
            self.move_forward()

    def move_forward(self):
        # Command to move forward
        tw_msg = Twist()
        tw_msg.linear.x = 0.3  # Forward speed
        tw_msg.angular.z = 0.25  # Rotation
        self.pub_cmd_vel.publish(tw_msg)
        #self.get_logger().info('Moving forward at constant speed.')

    def stop_moving(self):
        # Command to stop moving
        tw_msg = Twist()
        tw_msg.linear.x = -0.4  # No forward speed
        tw_msg.angular.z = 1.0  # No rotation
        self.pub_cmd_vel.publish(tw_msg)
        #self.get_logger().info('Adjusting...')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
