#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gps_driver");
  RCLCPP_INFO(node->get_logger(), "gps_driver starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
