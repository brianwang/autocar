#include "autocar_control/chassis_protocol.hpp"
#include <cstring>

namespace autocar_control {

// ── helpers ──
static void writeFloatLE(uint8_t* dst, float val) {
  uint32_t raw;
  static_assert(sizeof(raw) == sizeof(val), "float must be 32-bit");
  std::memcpy(&raw, &val, sizeof(raw));
  dst[0] = static_cast<uint8_t>(raw & 0xFF);
  dst[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  dst[2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  dst[3] = static_cast<uint8_t>((raw >> 24) & 0xFF);
}

static float readFloatLE(const uint8_t* src) {
  uint32_t raw = static_cast<uint32_t>(src[0])
               | (static_cast<uint32_t>(src[1]) << 8)
               | (static_cast<uint32_t>(src[2]) << 16)
               | (static_cast<uint32_t>(src[3]) << 24);
  float val;
  std::memcpy(&val, &raw, sizeof(val));
  return val;
}

// ── API ──

uint8_t computeChecksum(const uint8_t* data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

void encodeControlFrame(const ControlFrame& cmd, uint8_t* buffer) {
  // Byte 0: header
  buffer[0] = CONTROL_HEADER;
  // Byte 1: command type
  buffer[1] = CONTROL_CMD;
  // Bytes 2-5: vx (float, little-endian)
  writeFloatLE(buffer + 2, cmd.vx);
  // Bytes 6-9: wz (float, little-endian)
  writeFloatLE(buffer + 6, cmd.wz);
  // Byte 10: motor enable
  buffer[10] = cmd.motor_enable ? 0x01 : 0x00;
  // Bytes 11-14: reserved
  buffer[11] = 0x00;
  buffer[12] = 0x00;
  buffer[13] = 0x00;
  buffer[14] = 0x00;
  // Byte 15: checksum
  buffer[15] = computeChecksum(buffer, CONTROL_FRAME_SIZE - 1);
}

bool decodeFeedbackFrame(const uint8_t* buffer, FeedbackFrame& fb) {
  // Validate header
  if (buffer[0] != FEEDBACK_HEADER) return false;
  if (buffer[1] != FEEDBACK_CMD)   return false;

  // Validate checksum
  uint8_t expected_cs = computeChecksum(buffer, FEEDBACK_FRAME_SIZE - 1);
  if (buffer[FEEDBACK_FRAME_SIZE - 1] != expected_cs) return false;

  // Decode fields
  fb.left_mileage     = readFloatLE(buffer + 2);
  fb.right_mileage    = readFloatLE(buffer + 6);
  fb.battery_voltage  = readFloatLE(buffer + 10);
  fb.fault_code       = buffer[14];
  // bytes 15-23 reserved, ignored

  return true;
}

}  // namespace autocar_control
