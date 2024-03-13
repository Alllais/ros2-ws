import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.is_obstacle_distance = 0.5
        self.is_obstacle_detected = False

    def timer_callback(self):
        msg = String()
        msg.data = 'Interval: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        tw_msg = Twist()

        if not self.is_obstacle_detected:
            tw_msg.linear.x = 0.5
            tw_msg.angular.z = 0.0
            self.pub_cmd_vel.publish(tw_msg)
        else:
            tw_msg.angular.z = math.radians(90)
            self.pub_cmd_vel.publish(tw_msg)
            self.is_obstacle_detected = False

        self.i += 1

    def scan_callback(self, msg):
        # Simplified obstacle detection
        ranges = msg.ranges
        front_ranges = ranges[len(ranges)//3:len(ranges)*2//3]  # Consider only the front quadrant
        if any(distance < self.is_obstacle_distance for distance in front_ranges if distance > 0):
            self.get_logger().info('Obstacle detected!')
            self.is_obstacle_detected = True




def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
