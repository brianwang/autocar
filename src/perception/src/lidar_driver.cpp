#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lidar_driver");
  RCLCPP_INFO(node->get_logger(), "lidar_driver starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
