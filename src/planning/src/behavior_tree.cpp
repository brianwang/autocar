#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include "autocar_planning/behavior_nodes.hpp"

class BehaviorTree : public rclcpp::Node {
public:
  BehaviorTree() : Node("behavior_tree") {
    voice_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/voice_command", 10, std::bind(&BehaviorTree::voiceCallback, this, std::placeholders::_1));
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/task_goal", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/bt_state", 10);

    RCLCPP_INFO(this->get_logger(), "behavior_tree initialized");
  }

private:
  void voiceCallback(const std_msgs::msg::String::SharedPtr cmd) {
    RCLCPP_INFO(this->get_logger(), "BT received voice: '%s'", cmd->data.c_str());

    if (cmd->data.find("NAVIGATE") != std::string::npos) {
      auto goal = geometry_msgs::msg::PoseStamped();
      goal.header.frame_id = "map";
      goal.header.stamp = this->now();
      goal.pose.position.x = 50.0;
      goal.pose.position.y = 0.0;
      goal_pub_->publish(goal);

      auto state = std_msgs::msg::String();
      state.data = "navigating";
      state_pub_->publish(state);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorTree>());
  rclcpp::shutdown();
  return 0;
}
