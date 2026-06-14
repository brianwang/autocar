#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("steering_controller");
  RCLCPP_INFO(node->get_logger(), "steering_controller starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
