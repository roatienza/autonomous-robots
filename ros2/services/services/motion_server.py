import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel
    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)


class MotionServiceServer(Node):
    def __init__(self):
        super().__init__('motion_service_server')
        self.srv = self.create_service(SetBool, 'motion_service', self.motion_callback)
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.toggle = True
        

    def motion_callback(self, request, response):
        twist = Twist()
        if request.data:
            twist.linear.x = 0.01 if self.toggle else -0.01
            info = "Moving forward..." if self.toggle else "Moving backward..."
            self.toggle = not self.toggle
        else:
            twist.linear.x = 0.0
            info = "Stopping..."
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

        self.get_logger().info(f'Incoming motion request: {info}')
        response = SetBool.Response()
        response.success = True
        response.message = info
        return response


def main():
    rclpy.init()
    motion_service = MotionServiceServer()
    rclpy.spin(motion_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
