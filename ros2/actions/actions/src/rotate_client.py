import time
import math
import rclpy
import sys
from rclpy.action import ActionClient
from rclpy.node import Node

from actions.action import Rotate
from geometry_msgs.msg import Twist


class RotateActionClient(Node):

    def __init__(self):
        super().__init__('rotate_action_client')
        self._action_client = ActionClient(self, Rotate, 'rotate')

    def send_goal(self, angle):
        goal_msg = Rotate.Goal()
        goal_msg.angle = angle
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: Success={0}'.format(result.success))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_angle))

def main(angle, args=None):
    rclpy.init(args=args)
    rotate_action_client = RotateActionClient()
    future = rotate_action_client.send_goal(angle)
    rclpy.spin(rotate_action_client)
    #rclpy.spin_until_future_complete(rotate_action_client, future)


if __name__ == '__main__':
    if len(sys.argv) < 1:
        angle = 90.
    else:
        angle = float(sys.argv[1])
    main(angle=angle)
