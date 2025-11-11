from tutorial_interfaces.srv import AddThreeStrings  

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeStrings, 'add_three_strings', self.add_three_strings_callback)
        
    def add_three_strings_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                   # CHANGE
        self.get_logger().info('Incoming request\na: %s b: %s c: %s' % (request.a, request.b, request.c))  # CHANGE

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()