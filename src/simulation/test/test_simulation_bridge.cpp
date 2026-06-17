#include <gtest/gtest.h>
#include <string>
#include <map>
#include <random>
#include <algorithm>
#include <cmath>

// Topic mapping table
std::map<std::string, std::string> topic_map = {
  {"/model/autocar_car/cmd_vel", "/cmd_vel"},
  {"/model/autocar_car/odom", "/odom"},
  {"/model/autocar_car/scan", "/scan"},
  {"/model/autocar_car/imu", "/imu"},
  {"/model/autocar_car/gps", "/fix"},
  {"/model/autocar_car/depth_image", "/depth"},
  {"/model/autocar_car/tf", "/tf"},
};

// Noise configuration per sensor
struct NoiseConfig {
  double mean;
  double stddev;
};

std::map<std::string, NoiseConfig> noise_configs = {
  {"/model/autocar_car/scan", {0.0, 0.02}},
  {"/model/autocar_car/imu", {0.0, 0.01}},
  {"/model/autocar_car/gps", {0.0, 0.0001}},
};

TEST(BridgeTest, TopicMappingComplete) {
  EXPECT_EQ(topic_map["/model/autocar_car/cmd_vel"], "/cmd_vel");
  EXPECT_EQ(topic_map["/model/autocar_car/odom"], "/odom");
  EXPECT_EQ(topic_map["/model/autocar_car/scan"], "/scan");
  EXPECT_EQ(topic_map["/model/autocar_car/imu"], "/imu");
  EXPECT_EQ(topic_map["/model/autocar_car/gps"], "/fix");
  EXPECT_EQ(topic_map["/model/autocar_car/depth_image"], "/depth");
  EXPECT_EQ(topic_map["/model/autocar_car/tf"], "/tf");
}

TEST(BridgeTest, TopicCountMatchesSpec) {
  EXPECT_EQ(topic_map.size(), 7);
}

TEST(BridgeTest, NoiseConfigHasValidValues) {
  for (const auto& [topic, config] : noise_configs) {
    EXPECT_GE(config.stddev, 0.0) << topic << " should have non-negative stddev";
  }
}

// Simulated noise injection to test Gaussian clamping
TEST(BridgeTest, NoiseInjectionClampsToRange) {
  std::mt19937 rng(42);
  std::normal_distribution<double> nd(0.0, 0.02);
  double min_val = 0.05, max_val = 12.0;

  for (int i = 0; i < 100; ++i) {
    double original = 5.0;
    double noisy = std::clamp(original + nd(rng), min_val, max_val);
    EXPECT_GE(noisy, min_val);
    EXPECT_LE(noisy, max_val);
  }
}

// Command-line flag parsing
struct SimFlags {
  bool use_sim_time{false};
  bool noise_enabled{true};
};

SimFlags parseFlags(int argc, char** argv) {
  SimFlags flags;
  for (int i = 0; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg.find("use_sim_time:=true") != std::string::npos) flags.use_sim_time = true;
    if (arg.find("noise:=false") != std::string::npos) flags.noise_enabled = false;
  }
  return flags;
}

TEST(BridgeTest, ParseFlagsDefaults) {
  char* args[] = {const_cast<char*>("bridge")};
  auto flags = parseFlags(1, args);
  EXPECT_TRUE(flags.noise_enabled);
  EXPECT_FALSE(flags.use_sim_time);
}

TEST(BridgeTest, ParseFlagsSimTime) {
  char* args[] = {const_cast<char*>("bridge"), const_cast<char*>("use_sim_time:=true")};
  auto flags = parseFlags(2, args);
  EXPECT_TRUE(flags.use_sim_time);
}

TEST(BridgeTest, ParseFlagsNoiseDisabled) {
  char* args[] = {const_cast<char*>("bridge"), const_cast<char*>("noise:=false")};
  auto flags = parseFlags(2, args);
  EXPECT_FALSE(flags.noise_enabled);
}
