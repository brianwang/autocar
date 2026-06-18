#include <gtest/gtest.h>
#include "autocar_control/pid_controller.hpp"

TEST(PIDControllerTest, ProportionalOutput) {
  autocar_control::PIDController pid(2.0, 0.0, 0.0, -10.0, 10.0, 100.0);
  double output = pid.compute(5.0, 4.0, 1.0);
  EXPECT_NEAR(output, 2.0, 0.01);
}

TEST(PIDControllerTest, IntegralAccumulates) {
  autocar_control::PIDController pid(0.0, 1.0, 0.0, -10.0, 10.0, 100.0);
  pid.compute(1.0, 0.0, 1.0);
  double output = pid.compute(1.0, 0.0, 1.0);
  EXPECT_NEAR(output, 2.0, 0.01);
}

TEST(PIDControllerTest, DerivativeDampens) {
  autocar_control::PIDController pid(0.0, 0.0, 1.0, -10.0, 10.0, 100.0);
  pid.compute(2.0, 0.0, 1.0);
  double output = pid.compute(1.0, 1.0, 1.0);
  EXPECT_NEAR(output, -2.0, 0.01);
}

TEST(PIDControllerTest, OutputClamped) {
  autocar_control::PIDController pid(100.0, 0.0, 0.0, -3.0, 3.0, 100.0);
  double output = pid.compute(10.0, 0.0, 1.0);
  EXPECT_NEAR(output, 3.0, 0.01);
}

TEST(PIDControllerTest, ResetClearsIntegral) {
  autocar_control::PIDController pid(0.0, 1.0, 0.0, -10.0, 10.0, 100.0);
  pid.compute(1.0, 0.0, 1.0);
  pid.reset();
  double output = pid.compute(1.0, 0.0, 1.0);
  EXPECT_NEAR(output, 1.0, 0.01);
}

TEST(PIDControllerTest, IntegralAntiWindup) {
  autocar_control::PIDController pid(0.0, 1.0, 0.0, -2.0, 2.0, 3.0);
  for (int i = 0; i < 10; ++i) {
    pid.compute(10.0, 0.0, 1.0);
  }
  double output = pid.compute(10.0, 0.0, 1.0);
  EXPECT_NEAR(output, 2.0, 0.01);
}

TEST(PIDControllerTest, SetGainsReconfigures) {
  autocar_control::PIDController pid(1.0, 0.0, 0.0, -10.0, 10.0, 100.0);
  pid.setGains(3.0, 0.0, 0.0);
  double output = pid.compute(2.0, 1.0, 1.0);
  EXPECT_NEAR(output, 3.0, 0.01);
}
