from wozniak_interfaces.srv import PickObject

import rclpy
from rclpy.node import Node


class PickObjectService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(PickObject, 'pick_object', self.pick_object_callback)

    def pick_object_callback(self, request, response):
        response.success = True
        response.message = 'Object picked'
        self.get_logger().info('Incoming request\nObject: %s' % (request.object))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = PickObjectService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()