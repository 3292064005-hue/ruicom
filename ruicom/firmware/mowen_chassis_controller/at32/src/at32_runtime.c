#include "at32_runtime.h"

static uint16_t fw_crc16_ibm(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFU;
  size_t i = 0U;
  while (i < len) {
    crc ^= (uint16_t)data[i++];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      if ((crc & 0x0001U) != 0U) {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      } else {
        crc = (uint16_t)(crc >> 1U);
      }
    }
  }
  return crc;
}

static int16_t fw_le_i16(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8U);
  if (raw >= 0x8000U) {
    return (int16_t)(raw - 0x10000U);
  }
  return (int16_t)raw;
}

static void fw_push_u16(uint8_t *out, size_t *offset, uint16_t value) {
  out[(*offset)++] = (uint8_t)(value & 0xFFU);
  out[(*offset)++] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void fw_push_i16(uint8_t *out, size_t *offset, int16_t value) {
  fw_push_u16(out, offset, (uint16_t)value);
}

static int16_t fw_clamp_i16(int32_t value, int16_t minimum, int16_t maximum) {
  if (value < (int32_t)minimum) return minimum;
  if (value > (int32_t)maximum) return maximum;
  return (int16_t)value;
}

void fw_runtime_init(fw_runtime_state_t *state) {
  if (state == 0) return;
  state->last_command.vx_mm_s = 0;
  state->last_command.vy_mm_s = 0;
  state->last_command.wz_mrad_s = 0;
  state->last_command_ms = 0U;
  state->last_step_ms = 0U;
  for (uint8_t idx = 0U; idx < 4U; ++idx) {
    state->pid_integral[idx] = 0;
  }
}

void fw_runtime_apply_command(fw_runtime_state_t *state, const fw_velocity_command_t *cmd, uint32_t now_ms) {
  if (state == 0 || cmd == 0) return;
  state->last_command = *cmd;
  state->last_command_ms = now_ms;
}

static bool fw_command_timed_out(const fw_runtime_state_t *state, const fw_runtime_config_t *config, uint32_t now_ms) {
  if (state->last_command_ms == 0U) return true;
  return (now_ms - state->last_command_ms) > config->command_timeout_ms;
}

static fw_velocity_command_t fw_effective_command(const fw_runtime_state_t *state, const fw_runtime_config_t *config, bool estop, uint32_t now_ms) {
  fw_velocity_command_t cmd = {0, 0, 0};
  if (!estop && !fw_command_timed_out(state, config, now_ms)) {
    cmd = state->last_command;
  }
  return cmd;
}

static void fw_solve_mecanum_rpm(const fw_velocity_command_t *cmd, int16_t out_rpm[4]) {
  /* 50 mm wheel radius => ~191 RPM per m/s. k_mm ~= 280 mm. */
  const int32_t vx = (int32_t)cmd->vx_mm_s;
  const int32_t vy = (int32_t)cmd->vy_mm_s;
  const int32_t rot_term = (280 * (int32_t)cmd->wz_mrad_s) / 1000;
  const int32_t scale_num = 191;
  out_rpm[0] = fw_clamp_i16(((vx - vy - rot_term) * scale_num) / 1000, -3000, 3000);
  out_rpm[1] = fw_clamp_i16(((vx + vy + rot_term) * scale_num) / 1000, -3000, 3000);
  out_rpm[2] = fw_clamp_i16(((vx + vy - rot_term) * scale_num) / 1000, -3000, 3000);
  out_rpm[3] = fw_clamp_i16(((vx - vy + rot_term) * scale_num) / 1000, -3000, 3000);
}

static void fw_estimate_velocity_from_feedback(const fw_wheel_feedback_t *feedback, fw_telemetry_t *telemetry) {
  const int32_t mmps_per_rpm_num = 5236; /* 2*pi*50mm/60s scaled by 1000 */
  const int32_t mmps_per_rpm_den = 1000;
  int32_t wheel_mm_s[4];
  for (uint8_t idx = 0U; idx < 4U; ++idx) {
    wheel_mm_s[idx] = ((int32_t)feedback->rpm[idx] * mmps_per_rpm_num) / mmps_per_rpm_den;
  }
  telemetry->vx_mm_s = fw_clamp_i16((wheel_mm_s[0] + wheel_mm_s[1] + wheel_mm_s[2] + wheel_mm_s[3]) / 4, -3000, 3000);
  telemetry->vy_mm_s = fw_clamp_i16((-wheel_mm_s[0] + wheel_mm_s[1] + wheel_mm_s[2] - wheel_mm_s[3]) / 4, -3000, 3000);
  telemetry->wz_mrad_s = fw_clamp_i16(((-wheel_mm_s[0] + wheel_mm_s[1] - wheel_mm_s[2] + wheel_mm_s[3]) * 1000) / (4 * 280), -12000, 12000);
}

void fw_runtime_step(
    fw_runtime_state_t *state,
    const fw_runtime_config_t *config,
    const fw_wheel_feedback_t *feedback,
    uint16_t battery_mv,
    bool estop,
    fw_control_mode_t control_mode,
    bool obstacle_stop,
    uint32_t now_ms,
    uint16_t sequence,
    fw_runtime_result_t *out_result) {
  if (state == 0 || config == 0 || feedback == 0 || out_result == 0) return;
  fw_velocity_command_t effective = fw_effective_command(state, config, estop, now_ms);
  fw_solve_mecanum_rpm(&effective, out_result->target_rpm);
  uint32_t dt_ms = (state->last_step_ms == 0U) ? 20U : (now_ms - state->last_step_ms);
  if (dt_ms == 0U) dt_ms = 1U;
  for (uint8_t idx = 0U; idx < 4U; ++idx) {
    int32_t error = (int32_t)out_result->target_rpm[idx] - (int32_t)feedback->rpm[idx];
    state->pid_integral[idx] += error * (int32_t)dt_ms;
    if (state->pid_integral[idx] > 600000L) state->pid_integral[idx] = 600000L;
    if (state->pid_integral[idx] < -600000L) state->pid_integral[idx] = -600000L;
    int32_t drive = ((int32_t)config->pid_kp_milli * error) / 1000L + ((int32_t)config->pid_ki_milli * state->pid_integral[idx]) / 1000000L;
    if (estop) drive = 0;
    out_result->drive_permille[idx] = fw_clamp_i16(drive, (int16_t)-config->pid_output_limit_permille, (int16_t)config->pid_output_limit_permille);
  }
  out_result->telemetry.sequence = sequence;
  fw_estimate_velocity_from_feedback(feedback, &out_result->telemetry);
  out_result->telemetry.battery_mv = battery_mv == 0U ? config->nominal_battery_mv : battery_mv;
  out_result->telemetry.estop = estop;
  out_result->telemetry.heartbeat_ok = !estop;
  out_result->telemetry.control_mode = control_mode;
  out_result->telemetry.obstacle_stop = obstacle_stop;
  state->last_step_ms = now_ms;
}

bool fw_parse_legacy_frame(const uint8_t *frame, size_t len, fw_velocity_command_t *out_cmd) {
  if (frame == 0 || out_cmd == 0 || len != 12U) return false;
  if (!(frame[0] == 0xAAU && frame[1] == 0xBBU && frame[2] == 0x0AU && frame[3] == 0x12U && frame[4] == 0x02U)) {
    return false;
  }
  out_cmd->vx_mm_s = fw_le_i16(frame[5], frame[6]);
  out_cmd->vy_mm_s = fw_le_i16(frame[7], frame[8]);
  out_cmd->wz_mrad_s = fw_le_i16(frame[9], frame[10]);
  return true;
}

size_t fw_encode_managed_telemetry(const fw_telemetry_t *telemetry, uint8_t *out_frame, size_t out_capacity) {
  if (telemetry == 0 || out_frame == 0 || out_capacity < 19U) return 0U;
  size_t offset = 0U;
  fw_push_u16(out_frame, &offset, 0xAA55U);
  out_frame[offset++] = 0x81U;
  out_frame[offset++] = 11U;
  fw_push_u16(out_frame, &offset, telemetry->sequence);
  fw_push_i16(out_frame, &offset, telemetry->vx_mm_s);
  fw_push_i16(out_frame, &offset, telemetry->vy_mm_s);
  fw_push_i16(out_frame, &offset, telemetry->wz_mrad_s);
  fw_push_u16(out_frame, &offset, telemetry->battery_mv);
  uint8_t flags = 0U;
  if (telemetry->estop) flags |= 0x01U;
  if (telemetry->heartbeat_ok) flags |= 0x02U;
  flags |= (uint8_t)(((uint8_t)telemetry->control_mode & 0x03U) << 2U);
  if (telemetry->obstacle_stop) flags |= 0x10U;
  out_frame[offset++] = flags;
  fw_push_u16(out_frame, &offset, fw_crc16_ibm(&out_frame[2], offset - 2U));
  fw_push_u16(out_frame, &offset, 0x0D0AU);
  return offset;
}
