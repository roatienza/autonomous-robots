import rclpy
import time
import datetime
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class SensorsSubscriber(Node):
    def __init__(self):
        super().__init__('sensors_subscriber')
        
        self.set_params()
        self.get_params()
        self.scan_ranges = []
        self.init_scan_state = False

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery_state', self.battery_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.imu_count = 0

    def set_params(self):
        self.declare_parameter('timer_period', 10)

    def get_params(self):
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().integer_value

    def timer_callback(self):
        self.get_logger().info(str(datetime.datetime.now()))

    def scan_callback(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.init_scan_state = True
        self.scan_ranges = self.scan_ranges[self.scan_ranges > 0]
        min_range = np.amin(self.scan_ranges)
        max_range = np.amax(self.scan_ranges)
        info = f'Scan min range: {min_range}, max range: {max_range}'
        self.get_logger().info(info)

    def battery_callback(self, msg):
        present = msg.present
        voltage = msg.voltage  
        temp = msg.temperature
        current = msg.current
        info = f'battery: {present}, volt: {voltage}, temp: {temp}, current: {current}'
        self.get_logger().info(info)

    def imu_callback(self, msg):
        self.imu_count += 1
        if self.imu_count < 20:
            return
        self.imu_count = 0
        pitch = msg.orientation.x
        roll = msg.orientation.y
        
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        vx = msg.angular_velocity.x
        vy = msg.angular_velocity.y
        vz = msg.angular_velocity.z

        self.get_logger().info('%s: (pitch,roll): (%0.2f,%0.2f), (ax,ay,az): (%0.2f,%0.2f,%0.2f), (vx,vy,vz): (%0.2f,%0.2f,%0.2f) ' % \
                               (msg.header.frame_id, pitch, roll, ax, ay, az, vx, vy, vz))


def main(args=None):
    rclpy.init(args=args)
    sensors_subscriber = SensorsSubscriber()
    rclpy.spin(sensors_subscriber)
    sensors_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
