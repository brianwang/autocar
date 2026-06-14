"""Autocar web dashboard node."""
import rclpy
from rclpy.node import Node


class WebDashboard(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.get_logger().info('web_dashboard starting')


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
