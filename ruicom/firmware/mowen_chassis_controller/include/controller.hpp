#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include "protocol.hpp"

namespace mowen_fw {

struct WheelTargets {
  std::array<float, 4> rpm{};
};

struct WheelFeedback {
  std::array<float, 4> rpm{};
};

class PidAxis {
 public:
  explicit PidAxis(float kp = 0.4F, float ki = 0.02F, float kd = 0.0F, float output_limit = 1.0F)
      : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit) {}

  float step(float target, float measured, float dt_sec) {
    const float error = target - measured;
    integral_ += error * dt_sec;
    const float derivative = dt_sec > 1e-4F ? (error - prev_error_) / dt_sec : 0.0F;
    prev_error_ = error;
    float out = kp_ * error + ki_ * integral_ + kd_ * derivative;
    out = std::clamp(out, -output_limit_, output_limit_);
    return out;
  }

 private:
  float kp_{0.4F};
  float ki_{0.02F};
  float kd_{0.0F};
  float output_limit_{1.0F};
  float integral_{0.0F};
  float prev_error_{0.0F};
};

class MecanumController {
 public:
  MecanumController(float wheel_radius_m, float wheelbase_x_m, float wheelbase_y_m)
      : wheel_radius_m_(wheel_radius_m), wheelbase_x_m_(wheelbase_x_m), wheelbase_y_m_(wheelbase_y_m) {}

  WheelTargets solve(const VelocityCommand& cmd) const {
    const float vx = static_cast<float>(cmd.vx_mm_s) / 1000.0F;
    const float vy = static_cast<float>(cmd.vy_mm_s) / 1000.0F;
    const float wz = static_cast<float>(cmd.wz_mrad_s) / 1000.0F;
    const float k = wheelbase_x_m_ + wheelbase_y_m_;
    const float scale = 60.0F / (2.0F * static_cast<float>(M_PI) * wheel_radius_m_);
    WheelTargets targets;
    targets.rpm[0] = (vx - vy - k * wz) * scale;
    targets.rpm[1] = (vx + vy + k * wz) * scale;
    targets.rpm[2] = (vx + vy - k * wz) * scale;
    targets.rpm[3] = (vx - vy + k * wz) * scale;
    return targets;
  }

 private:
  float wheel_radius_m_{0.05F};
  float wheelbase_x_m_{0.15F};
  float wheelbase_y_m_{0.13F};
};

}  // namespace mowen_fw
