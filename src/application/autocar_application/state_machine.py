"""Autocar state machine node."""
import rclpy
from rclpy.node import Node


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.get_logger().info('state_machine starting')


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
