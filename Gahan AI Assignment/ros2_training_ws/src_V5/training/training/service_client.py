import sys
import rclpy
from rclpy.node import Node
from training_interfaces.srv import Value


class AddTwoIntsClient(Node):
    """Service client that sends two integers and prints the sum."""

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(Value, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service add_two_ints...')

    def send_request(self, a: int, b: int):
        request = Value.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()

    # Default values; override via CLI args: ros2 run training service_client 3 5
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    future = node.send_request(a, b)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.get_logger().info(f'Result: {a} + {b} = {result.val}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
