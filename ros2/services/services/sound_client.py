import sys
import rclpy
from rclpy.node import Node
from turtlebot3_msgs.srv import Sound

class SoundServiceClient(Node):
    def __init__(self):
        super().__init__('sound_service_client')
        self.cli = self.create_client(Sound, 'sound')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sound service not available, waiting again...')
        self.req = Sound.Request()

    def send_request(self, value):
        self.req.value = value
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    sound_client_service = SoundServiceClient()
    if len(sys.argv) < 2:
        info = "[Usage] ros2 run sensors_services sound_client <value>"
        sound_client_service.get_logger().info(info)
        info = "<value> : 0 to 5"
        sound_client_service.get_logger().info(info)
        sound_client_service.destroy_node()
        rclpy.shutdown()
        return
    response = sound_client_service.send_request(int(sys.argv[1]))
    value = int(sys.argv[1])
    info = f"Result of sound client for value = {value}: success={response.success}"
    info += f", message={response.message}"
    sound_client_service.get_logger().info(info)
    sound_client_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
