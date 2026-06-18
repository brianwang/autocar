#pragma once

#include <cstdint>
#include <cstddef>

namespace autocar_control {

// ── Protocol Constants ──
// Control frame: PC -> Chassis (16 bytes)
constexpr size_t   CONTROL_FRAME_SIZE   = 16;
constexpr uint8_t  CONTROL_HEADER       = 0xAA;
constexpr uint8_t  CONTROL_CMD          = 0x01;

// Feedback frame: Chassis -> PC (25 bytes)
constexpr size_t   FEEDBACK_FRAME_SIZE  = 25;
constexpr uint8_t  FEEDBACK_HEADER      = 0xBB;
constexpr uint8_t  FEEDBACK_CMD         = 0x02;

// ── Structured Data ──
struct ControlFrame {
  float vx{0.0f};           // linear velocity (m/s)
  float wz{0.0f};           // angular velocity (rad/s)
  bool  motor_enable{false};
};

struct FeedbackFrame {
  float   left_mileage{0.0f};     // left  wheel odometry (m)
  float   right_mileage{0.0f};    // right wheel odometry (m)
  float   battery_voltage{0.0f};  // battery voltage (V)
  uint8_t fault_code{0};          // 0 = no fault
};

// ── API ──

/**
 * Encode a ControlFrame into raw byte buffer.
 * buffer must be at least CONTROL_FRAME_SIZE bytes.
 */
void encodeControlFrame(const ControlFrame& cmd, uint8_t* buffer);

/**
 * Decode a FeedbackFrame from raw byte buffer.
 * Returns true on valid header + checksum.
 */
bool decodeFeedbackFrame(const uint8_t* buffer, FeedbackFrame& fb);

/**
 * Compute checksum: sum of bytes[0..len-1] masked to low 8 bits.
 */
uint8_t computeChecksum(const uint8_t* data, size_t len);

}  // namespace autocar_control
