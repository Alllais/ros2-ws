# An example of TurtleBot 3 subscribe to camera topic, mask colours, find and display contours, and move robot to center the object in image frame
# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan


class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')

        # publish cmd_vel topic to move the robot
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        # subscribe to the camera topic
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

        # subscription to the depth camera topic
        self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.depth_callback, 1)

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def depth_callback(self, depth_data):
        depth_frame = self.br.imgmsg_to_cv2(depth_data, desired_encoding='32FC1')
        threshold_distance = 0.5
        # print ("Checking if there is an object in front of the robot")
        if (depth_frame < threshold_distance).any():
            print("Object is close based on: depth camera")
        
        depth_frame_small = cv2.resize(depth_frame, (0,0), fx=0.6, fy=0.6) # reduce image size
        cv2.imshow("Depth Image", depth_frame_small)
        cv2.waitKey(1)

    def lidar_callback(self, scan_data):
        # Extract the ranges from the LaserScan message
        ranges = scan_data.ranges
        print("This is the min LIDAR range : ")
        print(min(ranges))
        min_distance = min(ranges)
        threshold_distance = 0.2
        self.is_obstacle_close = min_distance < threshold_distance

    def find_centroid(self, contours):
        centroid = None
        max_area_threshold = 50000
        if len(contours) > 0:
            for contour in contours:
                if max_area_threshold > cv2.contourArea(contour):
                    # Find the centroid of the contour
                    M = cv2.moments(contours[0])
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        print("Centroid of the biggest area: ({} ,{})".format(cx, cy))
                        centroid = (cx, cy)
        return centroid
    
    def movement(self, centroid, data):
        self.tw = Twist()
        if centroid:
            cx, _ = centroid
            if self.is_obstacle_close:
                print("Obstacle is close")
                self.tw.angular.z = 15.0
                self.tw.linear.x = -15.0
            elif cx < data.width / 3:
                self.tw.angular.z = 0.3
            elif cx >= 2 * data.width / 3:
                self.tw.angular.z = -0.3
            else:
                self.tw.angular.z = 0.0
                self.tw.linear.x = 100.0
        else:
            self.tw.angular.z = 0.3
        self.pub_cmd_vel.publish(self.tw)

    def camera_callback (self, data):

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Create a mask for range of colours (low , high)
        current_frame_mask = cv2.inRange(current_frame_hsv, (0, 150, 0), (255,255,255))
        
        contours, _ = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[:1]
        centroid = self.find_centroid(contours)
        self.movement(centroid, data)

        # Draw contours
        current_frame_contours = cv2.drawContours(current_frame.copy(), contours, 0, (255, 255, 0), 20)

        # Show the processed frame
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx = 0.4, fy = 0.4)
        cv2.imshow("Image Window", current_frame_contours_small)
        cv2.waitKey(1)
                

def main(args=None):
    print('Starting colour_chaser.py.')

    rclpy.init(args=args)

    colour_chaser = ColourChaser()

    rclpy.spin(colour_chaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

