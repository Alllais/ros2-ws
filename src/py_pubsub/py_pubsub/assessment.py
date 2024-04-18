import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        self.br = CvBridge()

        # LIDAR data to control movement
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.obstacle_distance_threshold = 0.3
        self.is_obstacle_detected = False

        # Camera data to process visual information
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

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

    def camera_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        current_frame_mask = cv2.inRange(current_frame_hsv, (0, 150, 50), (255, 255, 255))  # orange

        contours, _ = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        if contours:
            M = cv2.moments(contours[0])
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(current_frame, (cx, cy), 5, (0, 255, 0), -1)
                print("Centroid of the biggest area: ({}, {})".format(cx, cy))

        current_frame_contours_small = cv2.resize(current_frame, (0,0), fx = 0.4, fy = 0.4)
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)

    def move_forward(self):
        # Command to move forward
        tw_msg = Twist()
        tw_msg.linear.x = 0.4  # Forward speed
        tw_msg.angular.z = 0.25  # Rotation
        self.pub_cmd_vel.publish(tw_msg)
        #self.get_logger().info('Moving forward at constant speed.')

    def stop_moving(self):
        # Command to stop moving
        tw_msg = Twist()
        tw_msg.linear.x = -0.4  # No forward speed
        tw_msg.angular.z = 1.0  # No rotation
        self.pub_cmd_vel.publish(tw_msg)

def main(args=None):
    rclpy.init(args=args)
    combined_node = CombinedNode()
    rclpy.spin(combined_node)
    combined_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
