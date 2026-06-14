#pragma once

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <functional>

namespace autocar_planning {

struct AStarNode {
  int x, y;
  double g_cost{0.0};
  double h_cost{0.0};
  double f_cost() const { return g_cost + h_cost; }
  AStarNode* parent{nullptr};
  bool operator>(const AStarNode& other) const { return f_cost() > other.f_cost(); }
};

class AStarPlanner {
public:
  using OccupancyFn = std::function<bool(int, int)>;

  AStarPlanner(int width, int height, double resolution);

  void setOccupancyGrid(const std::vector<int8_t>& grid);
  void setOccupancyFn(OccupancyFn fn) { is_occupied_ = fn; }

  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped& start,
                            const geometry_msgs::msg::PoseStamped& goal);
  bool planToPath(const geometry_msgs::msg::PoseStamped& start,
                  const geometry_msgs::msg::PoseStamped& goal,
                  nav_msgs::msg::Path& path);

private:
  double heuristic(int x1, int y1, int x2, int y2) const;
  std::vector<std::pair<int, int>> getNeighbors(int x, int y) const;
  bool isValid(int x, int y) const;
  void worldToGrid(double wx, double wy, int& gx, int& gy) const;
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  int width_, height_;
  double resolution_;
  OccupancyFn is_occupied_;
};

}  // namespace autocar_planning
