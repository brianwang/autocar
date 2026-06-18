#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

TEST(MotorDriverTest, EncoderToOdom) {
  int left_ticks = 48;
  int right_ticks = 48;
  double wheel_radius = 0.032;
  double wheel_base = 0.178;
  int encoder_ppr = 48;

  double left_dist = (left_ticks / static_cast<double>(encoder_ppr)) * 2.0 * M_PI * wheel_radius;
  double right_dist = (right_ticks / static_cast<double>(encoder_ppr)) * 2.0 * M_PI * wheel_radius;

  double linear = (left_dist + right_dist) / 2.0;
  double angular = (right_dist - left_dist) / wheel_base;

  EXPECT_NEAR(linear, 2.0 * M_PI * wheel_radius, 0.001);
  EXPECT_NEAR(angular, 0.0, 0.001);
}

TEST(MotorDriverTest, DifferentialTurningOdom) {
  int left_ticks = 24;
  int right_ticks = 72;
  double wheel_radius = 0.032;
  double wheel_base = 0.178;
  int encoder_ppr = 48;

  double left_dist = (left_ticks / static_cast<double>(encoder_ppr)) * 2.0 * M_PI * wheel_radius;
  double right_dist = (right_ticks / static_cast<double>(encoder_ppr)) * 2.0 * M_PI * wheel_radius;

  double angular = (right_dist - left_dist) / wheel_base;
  EXPECT_GT(angular, 0.0);
}

TEST(MotorDriverTest, PWMClamped) {
  int pwm_value = 300;
  int clamped = std::clamp(pwm_value, -255, 255);
  EXPECT_EQ(clamped, 255);
}
