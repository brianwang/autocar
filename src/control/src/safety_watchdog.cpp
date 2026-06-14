#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("safety_watchdog");
  RCLCPP_INFO(node->get_logger(), "safety_watchdog starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
