import rclpy
from rclpy.node import Node
from training_interfaces.msg import Person


class PersonPublisher(Node):
    """Publishes Person messages at 1 Hz."""

    def __init__(self):
        super().__init__('person_publisher')
        self.publisher_ = self.create_publisher(Person, 'person_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('PersonPublisher started')

    def timer_callback(self):
        msg = Person()
        msg.name = 'Alice'
        msg.age = 30
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: name={msg.name}, age={msg.age}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
