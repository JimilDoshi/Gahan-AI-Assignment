import rclpy
from rclpy.node import Node
from training_interfaces.srv import Value


class AddTwoIntsServer(Node):
    """Service server that adds two integers and returns the sum."""

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(Value, 'add_two_ints', self.add_callback)
        self.get_logger().info('AddTwoIntsServer ready')

    def add_callback(self, request: Value.Request, response: Value.Response):
        response.val = request.a + request.b
        self.get_logger().info(
            f'Request: a={request.a}, b={request.b} -> val={response.val}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
