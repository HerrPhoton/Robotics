import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 
from std_msgs.msg import String


class ControllerNode(Node):

    def __init__(self):
        super().__init__('text_to_cmd_vel')     
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(String, 'cmd_text', self.convert2Twist, 10)

    def convert2Twist(self, msg):

        twist = Twist()

        if msg.data == 'turn_right':
            twist.angular.z = -1.57

        elif msg.data == 'turn_left':
            twist.angular.z = 1.57

        elif msg.data == 'move_forward':
            twist.linear.x = 1.0

        elif msg.data == 'move_backward':
            twist.linear.x = -1.0

        self.pub.publish(twist)

        
def main(args=None):
    rclpy.init(args=args)

    publisher = ControllerNode()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()