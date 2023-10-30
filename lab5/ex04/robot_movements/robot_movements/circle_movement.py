from math import pi

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Circle_publisher(Node):

    def __init__(self):
        super().__init__('Circle_publisher')

        self.pub = self.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10)

        self.timer = self.create_timer(1.0, self.publisher_callback)

    def publisher_callback(self):
        twist = Twist()

        twist.linear.x = 2 * pi
        twist.angular.z = 2 * pi

        self.pub.publish(twist)

def main():
    rclpy.init()

    node = Circle_publisher()
    rclpy.spin(node)

    rclpy.shutdown()