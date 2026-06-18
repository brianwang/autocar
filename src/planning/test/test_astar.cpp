#include <gtest/gtest.h>
#include "autocar_planning/astar.hpp"

TEST(AStarTest, StraightLinePath) {
  autocar_planning::AStarPlanner planner(10, 10, 1.0);
  planner.setOccupancyFn([](int, int) { return false; });

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0; start.pose.position.y = 0.0;
  goal.pose.position.x = 5.0; goal.pose.position.y = 0.0;

  auto path = planner.plan(start, goal);
  EXPECT_GT(path.poses.size(), 0u);
  EXPECT_NEAR(path.poses.back().pose.position.x, 5.0, 1.0);
}

TEST(AStarTest, BlockedPathGoesAround) {
  autocar_planning::AStarPlanner planner(10, 10, 1.0);
  planner.setOccupancyFn([](int x, int y) { return x == 1 && y == 0; });

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0; start.pose.position.y = 0.0;
  goal.pose.position.x = 5.0; goal.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  bool succ = planner.planToPath(start, goal, path);
  EXPECT_TRUE(succ);
  EXPECT_GT(path.poses.size(), 0u);
}

TEST(AStarTest, NoPathWhenFullyBlocked) {
  autocar_planning::AStarPlanner planner(3, 1, 1.0);
  planner.setOccupancyFn([](int, int) { return true; });

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0; start.pose.position.y = 0.0;
  goal.pose.position.x = 2.0; goal.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  bool succ = planner.planToPath(start, goal, path);
  EXPECT_FALSE(succ);
}
