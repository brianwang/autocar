#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("local_planner");
  RCLCPP_INFO(node->get_logger(), "local_planner starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
