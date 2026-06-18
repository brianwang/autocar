#include <gtest/gtest.h>
#include "autocar_control/pid_controller.hpp"

TEST(VelocityControllerTest, AcceleratesFromZero) {
  autocar_control::PIDController pid(1.2, 0.3, 0.05, -0.5, 0.5, 10.0);
  double cmd = pid.compute(0.3, 0.0, 0.05);
  EXPECT_GT(cmd, 0.0);
}

TEST(VelocityControllerTest, DeceleratesWhenOverspeed) {
  autocar_control::PIDController pid(1.2, 0.3, 0.05, -0.5, 0.5, 10.0);
  double cmd = pid.compute(0.3, 0.5, 0.05);
  EXPECT_LT(cmd, 0.0);
}

TEST(VelocityControllerTest, RespectsAccelLimit) {
  double max_accel = 0.3;
  double prev_cmd = 0.0;
  double cmd = 0.5;
  double accel = std::abs(cmd - prev_cmd) / 0.05;
  cmd = accel > max_accel ? prev_cmd + std::copysign(max_accel * 0.05, cmd - prev_cmd) : cmd;
  EXPECT_NEAR(cmd, 0.015, 0.001);
}
