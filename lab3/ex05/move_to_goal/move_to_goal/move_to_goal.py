from math import atan2, pi, sqrt
import time
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose


class Move2Goal(Node):

    def __init__(self, x, y, theta):
        super().__init__('move_to_goal')
        
        self.target_pos = Pose(x = float(x), y = float(y), theta = self.to_rads(float(theta)))
        self.start_pos = Pose()

        self.pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)

        self.sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

        self.get_logger().info(
            f'Moving the turtle to the coordinates: x = {self.target_pos.x:.2f}, y = {self.target_pos.y:.2f}, theta = {self.target_pos.theta:.2f}')

    def pose_callback(self, msg):
        self.start_pos = msg
        self.move2goal()

    def move2goal(self):

        angle1 = atan2(self.target_pos.y - self.start_pos.y,
                       self.target_pos.x - self.start_pos.x)
        
        angle1 -= self.start_pos.theta

        if angle1 > pi:
            angle1 -= 2 * pi
        if angle1 < -pi:
            angle1 += 2 * pi

        distance = sqrt((self.target_pos.y - self.start_pos.y) ** 2 +
                        (self.target_pos.x - self.start_pos.x) ** 2)
        
        angle2 = self.target_pos.theta - self.start_pos.theta - angle1

        if angle2 > pi:
            angle2 -= 2 * pi
        if angle2 < -pi:
            angle2 += 2 * pi

        # if angle2 > pi:
        #     angle2 -= 2 * pi
        # if angle2 < -pi:
        #     angle2 += 2 * pi

        twists = [Twist(angular = Vector3(z = angle1)), 
                  Twist(linear  = Vector3(x = distance)),
                  Twist(angular = Vector3(z = angle2))]

        for twist in twists:
            self.pub.publish(twist)
            time.sleep(1.0)

    def to_rads(self, angle):
        return angle * pi / 180



def main():
    rclpy.init()

    node = Move2Goal(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin_once(node)


if __name__ == '__main__':
    main()