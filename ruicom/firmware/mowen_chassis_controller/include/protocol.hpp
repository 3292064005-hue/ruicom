#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mowen_fw {

struct VelocityCommand {
  int16_t vx_mm_s{0};
  int16_t vy_mm_s{0};
  int16_t wz_mrad_s{0};
};

struct TelemetryFrame {
  uint16_t sequence{0};
  int16_t vx_mm_s{0};
  int16_t vy_mm_s{0};
  int16_t wz_mrad_s{0};
  uint16_t battery_mv{0};
  bool estop{false};
  bool heartbeat_ok{false};
};

inline uint16_t crc16_ibm(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      else crc = static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

inline int16_t le_i16(const uint8_t lo, const uint8_t hi) {
  return static_cast<int16_t>(static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
}

inline void push_le_u16(std::vector<uint8_t>& out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

inline void push_le_i16(std::vector<uint8_t>& out, int16_t value) {
  push_le_u16(out, static_cast<uint16_t>(value));
}

inline std::optional<VelocityCommand> parse_legacy_velocity_frame(const std::vector<uint8_t>& frame) {
  if (frame.size() != 12) return std::nullopt;
  if (!(frame[0] == 0xAA && frame[1] == 0xBB && frame[2] == 0x0A && frame[3] == 0x12 && frame[4] == 0x02)) return std::nullopt;
  VelocityCommand cmd;
  cmd.vx_mm_s = le_i16(frame[5], frame[6]);
  cmd.vy_mm_s = le_i16(frame[7], frame[8]);
  cmd.wz_mrad_s = le_i16(frame[9], frame[10]);
  return cmd;
}

inline std::vector<uint8_t> encode_managed_telemetry(const TelemetryFrame& telemetry) {
  constexpr uint16_t kSof = 0xAA55;
  constexpr uint16_t kEof = 0x0D0A;
  constexpr uint8_t kType = 0x81;
  std::vector<uint8_t> payload;
  push_le_u16(payload, telemetry.sequence);
  push_le_i16(payload, telemetry.vx_mm_s);
  push_le_i16(payload, telemetry.vy_mm_s);
  push_le_i16(payload, telemetry.wz_mrad_s);
  push_le_u16(payload, telemetry.battery_mv);
  uint8_t flags = 0;
  if (telemetry.estop) flags |= 0x01;
  if (telemetry.heartbeat_ok) flags |= 0x02;
  payload.push_back(flags);
  std::vector<uint8_t> frame;
  push_le_u16(frame, kSof);
  frame.push_back(kType);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16_ibm(frame.data() + 2, frame.size() - 2);
  push_le_u16(frame, crc);
  push_le_u16(frame, kEof);
  return frame;
}

}  // namespace mowen_fw
