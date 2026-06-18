#include <gtest/gtest.h>
#include "autocar_control/chassis_protocol.hpp"
#include <cstring>

using namespace autocar_control;

// ── encodeControlFrame ──

TEST(ChassisProtocolTest, EncodeControlHeader) {
  uint8_t buf[CONTROL_FRAME_SIZE]{};
  ControlFrame cmd;
  cmd.vx = 0.0f; cmd.wz = 0.0f; cmd.motor_enable = false;
  encodeControlFrame(cmd, buf);

  EXPECT_EQ(buf[0], 0xAA);
  EXPECT_EQ(buf[1], 0x01);
}

TEST(ChassisProtocolTest, EncodeControlVxWz) {
  uint8_t buf[CONTROL_FRAME_SIZE]{};
  ControlFrame cmd;
  cmd.vx = 0.5f; cmd.wz = -0.3f; cmd.motor_enable = true;
  encodeControlFrame(cmd, buf);

  // Read back floats
  float vx_out, wz_out;
  std::memcpy(&vx_out, buf + 2, sizeof(vx_out));
  std::memcpy(&wz_out, buf + 6, sizeof(wz_out));
  EXPECT_FLOAT_EQ(vx_out, 0.5f);
  EXPECT_FLOAT_EQ(wz_out, -0.3f);
}

TEST(ChassisProtocolTest, EncodeControlEnableFlag) {
  uint8_t buf[CONTROL_FRAME_SIZE]{};
  ControlFrame cmd;
  cmd.motor_enable = true;
  encodeControlFrame(cmd, buf);
  EXPECT_EQ(buf[10], 0x01);

  cmd.motor_enable = false;
  encodeControlFrame(cmd, buf);
  EXPECT_EQ(buf[10], 0x00);
}

TEST(ChassisProtocolTest, EncodeControlReservedZero) {
  uint8_t buf[CONTROL_FRAME_SIZE]{};
  ControlFrame cmd{};
  encodeControlFrame(cmd, buf);
  EXPECT_EQ(buf[11], 0x00);
  EXPECT_EQ(buf[12], 0x00);
  EXPECT_EQ(buf[13], 0x00);
  EXPECT_EQ(buf[14], 0x00);
}

TEST(ChassisProtocolTest, EncodeControlChecksumCorrect) {
  uint8_t buf[CONTROL_FRAME_SIZE]{};
  ControlFrame cmd;
  cmd.vx = 0.2f; cmd.wz = 0.0f; cmd.motor_enable = true;
  encodeControlFrame(cmd, buf);

  uint8_t expected = computeChecksum(buf, CONTROL_FRAME_SIZE - 1);
  EXPECT_EQ(buf[15], expected);
}

TEST(ChassisProtocolTest, EncodeControlVerifyFullFrame) {
  // Example: forward 0.2 m/s, no rotation, enabled
  uint8_t buf[CONTROL_FRAME_SIZE]{};
  ControlFrame cmd;
  cmd.vx = 0.2f; cmd.wz = 0.0f; cmd.motor_enable = true;
  encodeControlFrame(cmd, buf);

  EXPECT_EQ(buf[0], 0xAA);
  EXPECT_EQ(buf[1], 0x01);
  // vx = 0.2f LE
  float vx_dec;
  std::memcpy(&vx_dec, buf + 2, 4);
  EXPECT_FLOAT_EQ(vx_dec, 0.2f);
  // wz = 0.0f LE
  float wz_dec;
  std::memcpy(&wz_dec, buf + 6, 4);
  EXPECT_FLOAT_EQ(wz_dec, 0.0f);
  EXPECT_EQ(buf[10], 0x01);
  // reserved 0x00
  for (int i = 11; i <= 14; ++i) EXPECT_EQ(buf[i], 0x00);
  // checksum
  uint8_t cs = computeChecksum(buf, 15);
  EXPECT_EQ(buf[15], cs);
}

// ── decodeFeedbackFrame ──

TEST(ChassisProtocolTest, DecodeFeedbackValid) {
  uint8_t buf[FEEDBACK_FRAME_SIZE]{};
  buf[0] = 0xBB;
  buf[1] = 0x02;
  // left_mileage = 1.5f
  float lm = 1.5f;
  std::memcpy(buf + 2, &lm, 4);
  // right_mileage = 1.5f
  float rm = 1.5f;
  std::memcpy(buf + 6, &rm, 4);
  // battery = 12.5f
  float bat = 12.5f;
  std::memcpy(buf + 10, &bat, 4);
  buf[14] = 0x00; // no fault
  // reserved already 0x00
  buf[24] = computeChecksum(buf, FEEDBACK_FRAME_SIZE - 1);

  FeedbackFrame fb{};
  bool ok = decodeFeedbackFrame(buf, fb);
  EXPECT_TRUE(ok);
  EXPECT_FLOAT_EQ(fb.left_mileage, 1.5f);
  EXPECT_FLOAT_EQ(fb.right_mileage, 1.5f);
  EXPECT_FLOAT_EQ(fb.battery_voltage, 12.5f);
  EXPECT_EQ(fb.fault_code, 0x00);
}

TEST(ChassisProtocolTest, DecodeFeedbackWrongHeader) {
  uint8_t buf[FEEDBACK_FRAME_SIZE]{};
  buf[0] = 0xAA; // wrong
  buf[1] = 0x02;

  FeedbackFrame fb{};
  EXPECT_FALSE(decodeFeedbackFrame(buf, fb));
}

TEST(ChassisProtocolTest, DecodeFeedbackWrongCmd) {
  uint8_t buf[FEEDBACK_FRAME_SIZE]{};
  buf[0] = 0xBB;
  buf[1] = 0x01; // wrong

  FeedbackFrame fb{};
  EXPECT_FALSE(decodeFeedbackFrame(buf, fb));
}

TEST(ChassisProtocolTest, DecodeFeedbackBadChecksum) {
  uint8_t buf[FEEDBACK_FRAME_SIZE]{};
  buf[0] = 0xBB;
  buf[1] = 0x02;
  buf[24] = 0xFF; // bad checksum

  FeedbackFrame fb{};
  EXPECT_FALSE(decodeFeedbackFrame(buf, fb));
}

TEST(ChassisProtocolTest, DecodeFeedbackFaultCode) {
  uint8_t buf[FEEDBACK_FRAME_SIZE]{};
  buf[0] = 0xBB;
  buf[1] = 0x02;
  buf[14] = 0x03; // fault: motor overcurrent
  buf[24] = computeChecksum(buf, FEEDBACK_FRAME_SIZE - 1);

  FeedbackFrame fb{};
  EXPECT_TRUE(decodeFeedbackFrame(buf, fb));
  EXPECT_EQ(fb.fault_code, 0x03);
}

// ── computeChecksum ──

TEST(ChassisProtocolTest, ChecksumAccumulates) {
  uint8_t data[] = {0x01, 0x02, 0x03};
  EXPECT_EQ(computeChecksum(data, 3), 0x06);
}

TEST(ChassisProtocolTest, ChecksumWrapsLowByte) {
  uint8_t data[] = {0xFF, 0x01};
  EXPECT_EQ(computeChecksum(data, 2), 0x00); // 0x100 -> 0x00
}

TEST(ChassisProtocolTest, ChecksumVariableLen) {
  uint8_t data[] = {0xAA, 0x01, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                    0x00, 0x00, 0x00};
  // sum of above 15 bytes
  EXPECT_EQ(computeChecksum(data, 15), 0xAC);
}
