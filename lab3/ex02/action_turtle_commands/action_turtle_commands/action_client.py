import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_turtle_interface.action import MessageTurtleCommands


class TurtleSimActionClient(Node):

    def __init__(self):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, 
                                           MessageTurtleCommands, 
                                           'MessageTurtleCommands')

    def send_goal(self, goal):
        goal_msg = MessageTurtleCommands.Goal()

        goal_msg.command = goal['cmd']

        if goal.get('s') is None:
            goal_msg.s = 0.0
        else:
            goal_msg.s = goal['s']

        if goal.get('angle') is None:
            goal_msg.angle = 0.0
        else:
            goal_msg.angle = goal['angle']

        self._action_client.wait_for_server()

        self.get_logger().info(f'\nSending goal:\ncommand: {goal_msg.command}\ns: {goal_msg.s}\nangle: {goal_msg.angle}')

        return self._action_client.send_goal_async(goal_msg, self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance covered: {feedback.odom}')
            
def main():
    rclpy.init()

    action_client = TurtleSimActionClient()

    commands = [
        {'cmd' : 'forward', 's' : 2.0},
        {'cmd' : 'turn_right', 'angle' : 90.0},
        {'cmd' : 'forward', 's' : 1.0}]

    for command in commands:
        action_client.send_goal(command)
    
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()