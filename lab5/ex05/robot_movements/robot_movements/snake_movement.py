from math import sin, cos, fabs

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Snake_publisher(Node):

    def __init__(self):
        super().__init__('snake_publisher')

        self.pub = self.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10)

        self.timer = self.create_timer(1.0, self.publisher_callback)

    def publisher_callback(self):
        twist = Twist()
        time, _ = self.get_clock().now().seconds_nanoseconds()

        twist.linear.x = 2 * fabs(cos(time)) 
        twist.angular.z = 2 * sin(time)

        self.pub.publish(twist)

def main():
    rclpy.init()

    node = Snake_publisher()
    rclpy.spin(node)

    rclpy.shutdown()