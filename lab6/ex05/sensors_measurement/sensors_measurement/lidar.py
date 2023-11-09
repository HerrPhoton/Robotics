import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from numpy import min

class Forward_publisher(Node):

    def __init__(self):
        super().__init__('Forwar_publisher')

        self.pub = self.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10)
        
        self.sub = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.subscription_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.publisher_callback)
        self.is_obstacle = False

    def subscription_callback(self, msg):
        self.is_obstacle = min(msg.ranges) < 2.0

    def publisher_callback(self):
        twist = Twist()
        
        if not self.is_obstacle:
            twist.linear.x = 1.0

        self.pub.publish(twist)

def main():
    rclpy.init()

    node = Forward_publisher()
    rclpy.spin(node)

    rclpy.shutdown()