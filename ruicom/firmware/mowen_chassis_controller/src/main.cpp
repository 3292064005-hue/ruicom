#include <iostream>
#include <vector>

#include "runtime.hpp"

int main() {
  using namespace mowen_fw;
  std::vector<uint8_t> legacy = {0xAA, 0xBB, 0x0A, 0x12, 0x02, 0x64, 0x00, 0x32, 0x00, 0xF4, 0x01, 0x00};
  auto cmd = parse_legacy_velocity_frame(legacy);
  if (!cmd) {
    std::cerr << "legacy parse failed" << std::endl;
    return 2;
  }

  ChassisRuntime runtime(RuntimeConfig{});
  runtime.apply_command(*cmd, 10U);

  WheelFeedback measured{};
  measured.rpm = {0.0F, 0.0F, 0.0F, 0.0F};
  const RuntimeStepResult result = runtime.step(measured, 11800U, false, 30U, 1U);
  auto encoded = encode_managed_telemetry(result.telemetry);
  std::cout << "front_left_target_rpm=" << result.targets.rpm[0]
            << " drive=" << result.motor_drive[0]
            << " telemetry_bytes=" << encoded.size() << std::endl;
  return 0;
}
