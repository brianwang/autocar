#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("behavior_tree");
  RCLCPP_INFO(node->get_logger(), "behavior_tree starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
