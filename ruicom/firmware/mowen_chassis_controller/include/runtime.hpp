#pragma once

#include <array>
#include <cstdint>

#include "controller.hpp"
#include "protocol.hpp"

namespace mowen_fw {

struct RuntimeConfig {
  float wheel_radius_m{0.05F};
  float wheelbase_x_m{0.15F};
  float wheelbase_y_m{0.13F};
  float pid_kp{0.35F};
  float pid_ki{0.02F};
  float pid_kd{0.0F};
  float pid_output_limit{1.0F};
  uint32_t command_timeout_ms{500};
  uint16_t nominal_battery_mv{11800};
};

struct RuntimeStepResult {
  WheelTargets targets;
  std::array<float, 4> motor_drive{};
  TelemetryFrame telemetry;
};

class ChassisRuntime {
 public:
  explicit ChassisRuntime(const RuntimeConfig& config)
      : config_(config),
        controller_(config.wheel_radius_m, config.wheelbase_x_m, config.wheelbase_y_m),
        pid_axes_{
            PidAxis(config.pid_kp, config.pid_ki, config.pid_kd, config.pid_output_limit),
            PidAxis(config.pid_kp, config.pid_ki, config.pid_kd, config.pid_output_limit),
            PidAxis(config.pid_kp, config.pid_ki, config.pid_kd, config.pid_output_limit),
            PidAxis(config.pid_kp, config.pid_ki, config.pid_kd, config.pid_output_limit),
        } {}

  void apply_command(const VelocityCommand& cmd, uint32_t now_ms) {
    last_command_ = cmd;
    last_command_ms_ = now_ms;
  }

  RuntimeStepResult step(const WheelFeedback& measured, uint16_t battery_mv, bool estop, uint32_t now_ms, uint16_t sequence) {
    const VelocityCommand effective = effective_command(now_ms, estop);
    const WheelTargets targets = controller_.solve(effective);
    RuntimeStepResult result{};
    result.targets = targets;
    const float dt_sec = last_step_ms_ == 0 ? 0.02F : static_cast<float>(now_ms - last_step_ms_) / 1000.0F;
    for (size_t idx = 0; idx < result.motor_drive.size(); ++idx) {
      result.motor_drive[idx] = estop ? 0.0F : pid_axes_[idx].step(targets.rpm[idx], measured.rpm[idx], dt_sec);
    }
    result.telemetry.sequence = sequence;
    result.telemetry.vx_mm_s = effective.vx_mm_s;
    result.telemetry.vy_mm_s = effective.vy_mm_s;
    result.telemetry.wz_mrad_s = effective.wz_mrad_s;
    result.telemetry.battery_mv = battery_mv == 0 ? config_.nominal_battery_mv : battery_mv;
    result.telemetry.estop = estop;
    result.telemetry.heartbeat_ok = !estop && !command_timed_out(now_ms);
    last_step_ms_ = now_ms;
    return result;
  }

  bool command_timed_out(uint32_t now_ms) const {
    return last_command_ms_ == 0 || (now_ms - last_command_ms_) > config_.command_timeout_ms;
  }

 private:
  VelocityCommand effective_command(uint32_t now_ms, bool estop) const {
    if (estop || command_timed_out(now_ms)) {
      return VelocityCommand{};
    }
    return last_command_;
  }

  RuntimeConfig config_{};
  MecanumController controller_;
  std::array<PidAxis, 4> pid_axes_;
  VelocityCommand last_command_{};
  uint32_t last_command_ms_{0};
  uint32_t last_step_ms_{0};
};

}  // namespace mowen_fw
