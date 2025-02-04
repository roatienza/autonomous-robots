import rclpy
import datetime

from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        
        self.set_params()
        self.get_params()
        self.get_logger().info(f"Timer period: {self.period}")
        self.periodic = self.create_timer(self.period, self.periodic_callback)

    def set_params(self):
        self.declare_parameter('period', 1)

    def get_params(self):
        self.period = self.get_parameter('period').get_parameter_value().integer_value

    def periodic_callback(self):
        self.get_logger().info(str(datetime.datetime.now()))
        
def main(args=None):
    rclpy.init(args=args)
    periodic_node = TimerNode()
    rclpy.spin(periodic_node)
    periodic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
