#include "autocar_planning/dwa_planner.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace autocar_planning {

DWAPlanner::DWAPlanner(const DWAConfig& config) : config_(config) {}

geometry_msgs::msg::Twist DWAPlanner::plan(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::Twist& current_velocity,
    const nav_msgs::msg::Path& global_path,
    const sensor_msgs::msg::LaserScan& scan) {

  auto trajectories = generateTrajectories(current_pose, current_velocity);
  if (trajectories.empty()) {
    return geometry_msgs::msg::Twist();
  }

  for (auto& traj : trajectories) {
    traj.cost = evaluateTrajectory(traj, global_path, scan);
  }

  auto best = std::min_element(trajectories.begin(), trajectories.end(),
    [](const Trajectory& a, const Trajectory& b) { return a.cost < b.cost; });

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = best->linear_velocity;
  cmd.angular.z = best->angular_velocity;
  return cmd;
}

std::vector<Trajectory> DWAPlanner::generateTrajectories(
    const geometry_msgs::msg::PoseStamped&,
    const geometry_msgs::msg::Twist& vel) {
  std::vector<Trajectory> result;
  double v_min = std::max(0.0, vel.linear.x - config_.linear_accel * config_.predict_time);
  double v_max = std::min(config_.max_linear_speed, vel.linear.x + config_.linear_accel * config_.predict_time);
  double w_min = std::max(-config_.max_angular_speed, vel.angular.z - config_.angular_accel * config_.predict_time);
  double w_max = std::min(config_.max_angular_speed, vel.angular.z + config_.angular_accel * config_.predict_time);

  geometry_msgs::msg::PoseStamped origin;
  origin.pose.position.x = 0.0;
  origin.pose.position.y = 0.0;

  for (double v = v_min; v <= v_max; v += config_.linear_resolution) {
    for (double w = w_min; w <= w_max; w += config_.angular_resolution) {
      result.push_back(simulateTrajectory(v, w, origin));
    }
  }
  return result;
}

Trajectory DWAPlanner::simulateTrajectory(double v, double w,
    const geometry_msgs::msg::PoseStamped& start) {
  Trajectory traj;
  traj.linear_velocity = v;
  traj.angular_velocity = w;

  double x = start.pose.position.x;
  double y = start.pose.position.y;
  double theta = 0.0;

  for (double t = 0; t < config_.predict_time; t += config_.dt) {
    x += v * cos(theta) * config_.dt;
    y += v * sin(theta) * config_.dt;
    theta += w * config_.dt;

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    traj.poses.push_back(pose);
  }
  return traj;
}

double DWAPlanner::evaluateTrajectory(const Trajectory& traj,
    const nav_msgs::msg::Path& global_path,
    const sensor_msgs::msg::LaserScan& scan) {
  return config_.obstacle_cost_weight * obstacleCost(traj, scan) +
         config_.goal_cost_weight * goalCost(traj, global_path) +
         config_.speed_cost_weight * speedCost(traj);
}

double DWAPlanner::obstacleCost(const Trajectory& traj, const sensor_msgs::msg::LaserScan& scan) {
  double min_dist = std::numeric_limits<double>::max();
  for (const auto& pose : traj.poses) {
    double dist = std::hypot(pose.pose.position.x, pose.pose.position.y);
    if (dist < config_.goal_tolerance) return 1e9;
    min_dist = std::min(min_dist, dist);
  }
  return 1.0 / (min_dist + 0.1);
}

double DWAPlanner::goalCost(const Trajectory& traj, const nav_msgs::msg::Path& global_path) {
  if (global_path.poses.empty()) return 0.0;
  const auto& goal = global_path.poses.back();
  const auto& last = traj.poses.back();
  return std::hypot(last.pose.position.x - goal.pose.position.x,
                    last.pose.position.y - goal.pose.position.y);
}

double DWAPlanner::speedCost(const Trajectory& traj) {
  return config_.max_linear_speed - traj.linear_velocity;
}

}  // namespace autocar_planning
