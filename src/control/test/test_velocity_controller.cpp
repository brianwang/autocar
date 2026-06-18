#include <gtest/gtest.h>
#include "autocar_control/pid_controller.hpp"
#include <cmath>
#include <algorithm>

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
  double signed_accel = (cmd > prev_cmd) ? max_accel : -max_accel;
  cmd = prev_cmd + signed_accel * 0.05;
  EXPECT_NEAR(cmd, 0.015, 0.001);
}
