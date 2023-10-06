from math import atan2, pi, sqrt
import time
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Move2Goal(Node):

    def __init__(self, x, y, theta):
        super().__init__('move_to_goal')
        
        self.target_pos = Pose(x = float(x), y = float(y), theta = self.to_rads(float(theta)))
        self.cur_pos = Pose()

        self.pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)

        self.sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        self.timer = self.create_timer(0.1, self.move2goal)

        self.distance_error = 0.1
        self.print_coords = False

        self.get_logger().info(
            f'Moving the turtle to the coordinates:\nx = {self.target_pos.x:.2f},\ny = {self.target_pos.y:.2f},\ntheta = {self.target_pos.theta:.2f}')

    def pose_callback(self, msg):
        self.cur_pos = msg

        if self.print_coords:
            self.get_logger().info(
                f'Final coordinates:\nx = {msg.x:.2f},\ny = {msg.y:.2f},\ntheta = {msg.theta:.2f}')
            
            sys.exit()

    def move2goal(self):
        
        twist = Twist()

        # Radians to the target point
        angle1 = atan2(self.target_pos.y - self.cur_pos.y,
                       self.target_pos.x - self.cur_pos.x)
                
        angle1 -= self.cur_pos.theta
        angle1  = self.scaling_angle(angle1)

        # Distance to the target point
        distance = self.get_distance()

        twist.linear.x  = distance
        twist.angular.z = angle1

        # Radians to the target angle
        if distance <= self.distance_error:
            
            self.timer.destroy()

            angle2 = self.target_pos.theta - self.cur_pos.theta
            angle2 = self.scaling_angle(angle2)

            twist.linear.x  = 0.0
            twist.angular.z = angle2

            self.pub.publish(twist)

            self.print_coords = True
            time.sleep(1.0)

        else:
            self.pub.publish(twist)

    def get_distance(self):    
        return sqrt((self.target_pos.y - self.cur_pos.y) ** 2 +
                    (self.target_pos.x - self.cur_pos.x) ** 2)

    def scaling_angle(self, angle):

        if angle > pi:
            angle -= 2 * pi
        if angle < -pi:
            angle += 2 * pi

        return angle

    def to_rads(self, angle):
        return angle * pi / 180


def main():
    rclpy.init()

    node = Move2Goal(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin(node)

if __name__ == '__main__':
    main()