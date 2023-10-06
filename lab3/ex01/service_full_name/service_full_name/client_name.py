import sys

from name_interface.srv import FullNameService
import rclpy
from rclpy.node import Node


class Client(Node):

    def __init__(self):
        super().__init__('summ_full_name_client')
        self.client = self.create_client(FullNameService, 'summ_full_name')

        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = FullNameService.Request()

    def send_request(self, last_name, name, first_name):

        self.req.last_name = last_name
        self.req.name = name
        self.req.first_name = first_name

        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main():
    rclpy.init()

    client = Client()
    response = client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])

    client.get_logger().info(
        'Full_name: %s' % response.full_name)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
