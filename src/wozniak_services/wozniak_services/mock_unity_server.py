import rclpy
from rclpy.node import Node
from wozniak_interfaces.srv import Coord


class CoordService(Node):
    def __init__(self):
        super().__init__('coord_service')
        self.srv = self.create_service(Coord, '/Coord', self.handle_coord_request)
        self.get_logger().info('Coord service is ready.')

    def handle_coord_request(self, request, response):
        self.get_logger().info(f'Received request: x={request.x}, y={request.y}, z={request.z}')
        response.success = True
        return response


def main():
    rclpy.init()
    node = CoordService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
