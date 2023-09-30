from name_interface.srv import FullNameService

import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        super().__init__('summ_full_name_service')
        self.srv = self.create_service(FullNameService, 'summ_full_name', self.full_name_callback)

    def full_name_callback(self, request, response):

        response.full_name = '_'.join([
            request.last_name, 
            request.name, 
            request.first_name])
        
        self.get_logger().info(f'Incoming request: \
                               last_name: {request.last_name}, \
                               name: {request.name}, \
                               first_name: {request.first_name}')

        return response


def main():
    rclpy.init()

    minimal_service = Service()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
