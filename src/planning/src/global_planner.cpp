#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "autocar_planning/astar.hpp"

class GlobalPlanner : public rclcpp::Node {
public:
  GlobalPlanner() : Node("global_planner"), astar_(200, 200, 1.0) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task_goal", 10,
      std::bind(&GlobalPlanner::goalCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "global_planner initialized");
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.position.x = 0.0;
    start.pose.position.y = 0.0;

    auto path = astar_.plan(start, *goal);
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    path_pub_->publish(path);
  }

  autocar_planning::AStarPlanner astar_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
