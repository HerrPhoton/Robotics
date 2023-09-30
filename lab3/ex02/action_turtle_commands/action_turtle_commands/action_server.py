from math import pi, sqrt
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_turtle_interface.action import MessageTurtleCommands
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleSimActionServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')

        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'MessageTurtleCommands',
            self.execute_callback)
        
        self.pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)

        self.sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        self.steps = 3
        self.get_logger().info('Action turtle server has been initialised.')

    def pose_callback(self, data):
        pass

    def calculate_distance(self, point1, point2):

        self.get_logger().info(f'{point1}, {point2}')

        return sqrt((point1.x - point2.x) ** 2 + \
                    (point1.y - point2.y) ** 2)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        result_msg = MessageTurtleCommands.Result()
        feedback_msg = MessageTurtleCommands.Feedback()

        twist = Twist()

        cmd = goal_handle.request.command
        distance = goal_handle.request.s
        angle = goal_handle.request.angle

        for _ in range(1, self.steps + 1):

            if cmd == 'forward':
                
                twist.linear.x = distance / self.steps
                feedback_msg.odom += distance / self.steps

                goal_handle.publish_feedback(feedback_msg)

            elif cmd == 'turn_left':
                twist.angular.z = self.to_rads(angle / self.steps)

            elif cmd == 'turn_right':
                twist.angular.z = -self.to_rads(angle / self.steps)

            self.pub.publish(twist)
            time.sleep(1.0)

        goal_handle.succeed()
        result_msg.result = True

        return result_msg
    
    def to_rads(self, angle):
        return angle * pi / 180


def main():
    rclpy.init()

    turtlesimServer = TurtleSimActionServer()

    rclpy.spin(turtlesimServer)


if __name__ == '__main__':
    main()