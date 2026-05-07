import rclpy
from rclpy.node import Node
from training_interfaces.msg import Person


class PersonSubscriber(Node):
    """Subscribes to Person messages and logs them."""

    def __init__(self):
        super().__init__('person_subscriber')
        self.subscription = self.create_subscription(
            Person,
            'person_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info('PersonSubscriber started')

    def listener_callback(self, msg: Person):
        self.get_logger().info(f'Received: name={msg.name}, age={msg.age}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
