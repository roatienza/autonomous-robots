import time
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from actions.action import Rotate
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile


class RotateActionServer(Node):

    def __init__(self):
        super().__init__('rotate_action_server')
        self._action_server = ActionServer(
            self,
            Rotate,
            'rotate',
            self.execute_callback)
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Rotate.Feedback()
        feedback_msg.current_angle = 0.
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        angular_speed = 0.5
        if goal_handle.request.angle > 0:
            twist.angular.z = angular_speed
        else:
            twist.angular.z = -angular_speed
        self.pub.publish(twist)        

        target_rad = goal_handle.request.angle * math.pi/180.
        current_rad = 0.
        delay = .1
        while abs(current_rad) <= abs(target_rad):
            current_rad += twist.angular.z*delay
            feedback_msg.current_angle = current_rad * 180./math.pi
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_angle))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(delay)

        twist.angular.z = 0.0
        self.pub.publish(twist)        
        goal_handle.succeed()
        result = Rotate.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    rotate_action_server = RotateActionServer()
    rclpy.spin(rotate_action_server)


if __name__ == '__main__':
    main()
