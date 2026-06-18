#include "autocar_planning/astar.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace autocar_planning {

AStarPlanner::AStarPlanner(int width, int height, double resolution)
  : width_(width), height_(height), resolution_(resolution) {
  is_occupied_ = [](int, int) { return false; };
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const {
  return std::hypot(x1 - x2, y1 - y2) * resolution_;
}

std::vector<std::pair<int, int>> AStarPlanner::getNeighbors(int x, int y) const {
  std::vector<std::pair<int, int>> neighbors;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      neighbors.push_back({x + dx, y + dy});
    }
  }
  return neighbors;
}

bool AStarPlanner::isValid(int x, int y) const {
  return x >= 0 && x < width_ && y >= 0 && y < height_ && !is_occupied_(x, y);
}

void AStarPlanner::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  gx = static_cast<int>(wx / resolution_);
  gy = static_cast<int>(wy / resolution_);
}

void AStarPlanner::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  wx = gx * resolution_;
  wy = gy * resolution_;
}

nav_msgs::msg::Path AStarPlanner::plan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = start.header.frame_id;
  planToPath(start, goal, path);
  return path;
}

bool AStarPlanner::planToPath(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    nav_msgs::msg::Path& path) {
  int sx, sy, gx, gy;
  worldToGrid(start.pose.position.x, start.pose.position.y, sx, sy);
  worldToGrid(goal.pose.position.x, goal.pose.position.y, gx, gy);

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
  std::map<int, std::map<int, bool>> closed;

  AStarNode start_node;
  start_node.x = sx; start_node.y = sy;
  start_node.g_cost = 0.0;
  start_node.h_cost = heuristic(sx, sy, gx, gy);
  open.push(start_node);

  auto key = [](int x, int y) { return x * 65536 + y; };
  std::unordered_map<int, std::pair<int, int>> parent_map;
  std::unordered_map<int, double> g_cost_map;
  parent_map[key(sx, sy)] = {-1, -1};
  g_cost_map[key(sx, sy)] = 0.0;

  while (!open.empty()) {
    auto current = open.top(); open.pop();
    int ck = key(current.x, current.y);

    if (current.x == gx && current.y == gy) {
      // Reconstruct path
      std::vector<std::pair<int, int>> trace;
      int cx = gx, cy = gy;
      while (cx != -1 && cy != -1) {
        trace.push_back({cx, cy});
        int pk = key(cx, cy);
        auto it = parent_map.find(pk);
        if (it == parent_map.end()) break;
        cx = it->second.first;
        cy = it->second.second;
      }
      std::reverse(trace.begin(), trace.end());
      for (auto [px, py] : trace) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = start.header;
        gridToWorld(px, py, pose.pose.position.x, pose.pose.position.y);
        path.poses.push_back(pose);
      }
      return true;
    }

    closed[current.x][current.y] = true;

    for (auto [nx, ny] : getNeighbors(current.x, current.y)) {
      if (!isValid(nx, ny) || closed[nx][ny]) continue;
      double step_cost = (nx != current.x && ny != current.y) ? 1.414 : 1.0;
      double new_g = g_cost_map[ck] + step_cost;
      int nk = key(nx, ny);
      if (g_cost_map.find(nk) == g_cost_map.end() || new_g < g_cost_map[nk]) {
        g_cost_map[nk] = new_g;
        parent_map[nk] = {current.x, current.y};
        AStarNode neighbor;
        neighbor.x = nx; neighbor.y = ny;
        neighbor.g_cost = new_g;
        neighbor.h_cost = heuristic(nx, ny, gx, gy);
        open.push(neighbor);
      }
    }
  }
  return false;
}

}  // namespace autocar_planning
