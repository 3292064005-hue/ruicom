#include <cassert>

#include "runtime.hpp"

int main() {
  using namespace mowen_fw;
  ChassisRuntime runtime(RuntimeConfig{});
  VelocityCommand cmd{};
  cmd.vx_mm_s = 500;
  runtime.apply_command(cmd, 100U);

  WheelFeedback measured{};
  const RuntimeStepResult active = runtime.step(measured, 12000U, false, 120U, 2U);
  assert(active.telemetry.vx_mm_s == 500);
  assert(active.telemetry.heartbeat_ok);
  assert(active.targets.rpm[0] != 0.0F || active.targets.rpm[1] != 0.0F);

  const RuntimeStepResult stale = runtime.step(measured, 12000U, false, 800U, 3U);
  assert(stale.telemetry.vx_mm_s == 0);
  assert(!stale.telemetry.heartbeat_ok);

  const RuntimeStepResult estop = runtime.step(measured, 12000U, true, 820U, 4U);
  for (float value : estop.motor_drive) {
    assert(value == 0.0F);
  }
  assert(estop.telemetry.estop);
  return 0;
}
