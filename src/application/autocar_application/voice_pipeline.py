"""Autocar voice pipeline node."""
import rclpy
from rclpy.node import Node


class VoicePipeline(Node):
    def __init__(self):
        super().__init__('voice_pipeline')
        self.get_logger().info('voice_pipeline starting')


def main(args=None):
    rclpy.init(args=args)
    node = VoicePipeline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
