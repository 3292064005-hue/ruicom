#include <cassert>
#include <vector>
#include "protocol.hpp"

int main() {
  using namespace mowen_fw;
  std::vector<uint8_t> legacy = {0xAA, 0xBB, 0x0A, 0x12, 0x02, 0x64, 0x00, 0x9C, 0xFF, 0x2C, 0x01, 0x00};
  auto cmd = parse_legacy_velocity_frame(legacy);
  assert(cmd.has_value());
  assert(cmd->vx_mm_s == 100);
  assert(cmd->vy_mm_s == -100);
  assert(cmd->wz_mrad_s == 300);
  TelemetryFrame telemetry{};
  telemetry.sequence = 1;
  telemetry.vx_mm_s = cmd->vx_mm_s;
  telemetry.vy_mm_s = cmd->vy_mm_s;
  telemetry.wz_mrad_s = cmd->wz_mrad_s;
  telemetry.battery_mv = 11800;
  telemetry.estop = false;
  telemetry.heartbeat_ok = true;
  auto frame = encode_managed_telemetry(telemetry);
  assert(frame.size() > 8);
  return 0;
}
