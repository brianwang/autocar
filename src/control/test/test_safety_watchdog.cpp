#include <gtest/gtest.h>

enum class SafetyLevel { FATAL = 0, CRITICAL = 1, WARNING = 2, NORMAL = 3 };

SafetyLevel checkBattery(float voltage) {
  if (voltage < 10.5) return SafetyLevel::FATAL;
  if (voltage < 11.0) return SafetyLevel::CRITICAL;
  if (voltage < 11.5) return SafetyLevel::WARNING;
  return SafetyLevel::NORMAL;
}

SafetyLevel checkCmdVelTimeout(double last_cmd_time_ms, double now_ms) {
  if (now_ms - last_cmd_time_ms > 200.0) return SafetyLevel::CRITICAL;
  return SafetyLevel::NORMAL;
}

TEST(SafetyTest, BatteryLowIsCritical) {
  EXPECT_EQ(checkBattery(10.3), SafetyLevel::FATAL);
  EXPECT_EQ(checkBattery(10.7), SafetyLevel::CRITICAL);
  EXPECT_EQ(checkBattery(11.2), SafetyLevel::WARNING);
  EXPECT_EQ(checkBattery(12.0), SafetyLevel::NORMAL);
}

TEST(SafetyTest, CmdVelTimeoutTriggers) {
  EXPECT_EQ(checkCmdVelTimeout(0, 300), SafetyLevel::CRITICAL);
  EXPECT_EQ(checkCmdVelTimeout(100, 250), SafetyLevel::NORMAL);
}
