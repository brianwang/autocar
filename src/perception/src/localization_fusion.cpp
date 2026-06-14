#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("localization_fusion");
  RCLCPP_INFO(node->get_logger(), "localization_fusion starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
