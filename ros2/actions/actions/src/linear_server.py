import time
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from actions.action import Linear
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile


class LinearActionServer(Node):

    def __init__(self):
        super().__init__('linear_action_server')
        self._action_server = ActionServer(
            self,
            Linear,
            'linear',
            self.execute_callback)
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Linear.Feedback()
        feedback_msg.current_linear = 0.
        twist = Twist()
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        linear_speed = 0.1
        if goal_handle.request.linear > 0:
            twist.linear.x = linear_speed
        else:
            twist.linear.x = -linear_speed
        self.pub.publish(twist)        

        delay = .1
        current_linear = 0
        target_linear = goal_handle.request.linear
        while abs(current_linear) <= abs(target_linear):
            current_linear += twist.linear.x*delay
            feedback_msg.current_linear = current_linear
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_linear))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(delay)

        twist.linear.x = 0.0
        self.pub.publish(twist)        
        goal_handle.succeed()
        result = Linear.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    linear_action_server = LinearActionServer()
    rclpy.spin(linear_action_server)


if __name__ == '__main__':
    main()
