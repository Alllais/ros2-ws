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

        self.moving_forward = False

        # publish cmd_vel topic to move the robot
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        # subscribe to the camera topic
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

        # subscription to the depth camera topic
        self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.depth_callback, 1)

        #self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)

        self.current_centroid = None
        self.is_tracking = False

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def depth_callback(self, depth_data):
        depth_frame = self.br.imgmsg_to_cv2(depth_data, desired_encoding='32FC1')
        height, width = depth_frame.shape
        center_x, center_y = width // 2, height // 2
        self.center_depth = depth_frame[center_y, center_x]
        #print(f"Depth at the center: {self.center_depth}")
        if self.center_depth < 0.2:  # Update threshold as necessary
            print("Stopping due to close object detected by depth camera.")
            #self.moving_forward = False



    def find_centroid(self, contours, image_height=500):
        half_height = image_height / 2  # Calculate the midpoint of the height

        if self.is_tracking and not contours:
            # If tracking but no contours are found, continue using the old centroid
            return self.current_centroid

        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                if cy > half_height:
                    if not self.is_tracking:
                        print(f"New centroid locked: ({cx}, {cy})")
                    else:
                        print(f"Updating centroid position to: ({cx}, {cy})")
                    self.current_centroid = (cx, cy)
                    self.is_tracking = True
                    break
        return self.current_centroid




    
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





    def camera_callback (self, data):

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Create a mask for range of colours (low , high)
        lower_green = np.array([50, 100, 100])  # Adjust these values based on your needs
        upper_green = np.array([70, 255, 255])
        green_mask = cv2.inRange(current_frame_hsv, lower_green, upper_green)

        # Define HSV range for red (note: red wraps around the hue axis)
        lower_red1 = np.array([0, 110, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 110, 100])
        upper_red2 = np.array([180, 255, 255])
        red_mask1 = cv2.inRange(current_frame_hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(current_frame_hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Combine the green and red masks
        current_frame_mask = cv2.bitwise_or(green_mask, red_mask)
        
        contours, _ = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        height, width = current_frame.shape[:2]
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[:1]
        centroid = self.find_centroid(contours, height)
        #self.movement(centroid, data)

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

