#pragma once

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

namespace autocar_planning {

struct DWAConfig {
  double max_linear_speed{0.5};
  double max_angular_speed{1.5};
  double linear_accel{0.3};
  double angular_accel{0.8};
  double linear_resolution{0.05};
  double angular_resolution{0.1};
  double predict_time{2.0};
  double dt{0.1};
  double goal_tolerance{0.2};
  double obstacle_cost_weight{1.0};
  double goal_cost_weight{0.5};
  double speed_cost_weight{0.3};
};

struct Trajectory {
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  double linear_velocity;
  double angular_velocity;
  double cost;
};

class DWAPlanner {
public:
  explicit DWAPlanner(const DWAConfig& config = DWAConfig());

  geometry_msgs::msg::Twist plan(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::Twist& current_velocity,
    const nav_msgs::msg::Path& global_path,
    const sensor_msgs::msg::LaserScan& scan
  );

private:
  std::vector<Trajectory> generateTrajectories(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& vel
  );
  Trajectory simulateTrajectory(double linear_v, double angular_v,
    const geometry_msgs::msg::PoseStamped& start_pose);
  double evaluateTrajectory(const Trajectory& traj,
    const nav_msgs::msg::Path& global_path,
    const sensor_msgs::msg::LaserScan& scan);
  double obstacleCost(const Trajectory& traj, const sensor_msgs::msg::LaserScan& scan);
  double goalCost(const Trajectory& traj, const nav_msgs::msg::Path& global_path);
  double speedCost(const Trajectory& traj);

  DWAConfig config_;
};

}  // namespace autocar_planning
