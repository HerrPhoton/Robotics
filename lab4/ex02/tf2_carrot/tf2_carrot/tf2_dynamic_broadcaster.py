import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.radius = self.declare_parameter(
            'radius', 1.0).get_parameter_value().double_value
        
        self.dir_of_rot = self.declare_parameter(
            'direction_of_rotation', 1).get_parameter_value().integer_value

    def broadcast_timer_callback(self):
        seconds, _ = self.get_clock().now().seconds_nanoseconds()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        t.transform.translation.x = self.radius * math.cos(seconds * self.dir_of_rot)
        t.transform.translation.y = self.radius * math.sin(seconds * self.dir_of_rot)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()

    node = DynamicFrameBroadcaster()
    rclpy.spin(node)

    rclpy.shutdown()