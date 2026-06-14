#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("slam_node");
  RCLCPP_INFO(node->get_logger(), "slam_node starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
