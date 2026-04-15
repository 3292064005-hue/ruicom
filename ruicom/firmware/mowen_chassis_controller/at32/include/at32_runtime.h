#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  FW_CONTROL_MODE_COMMANDER = 0,
  FW_CONTROL_MODE_HANDLE = 1,
  FW_CONTROL_MODE_VOICE = 2,
} fw_control_mode_t;

typedef struct {
  int16_t vx_mm_s;
  int16_t vy_mm_s;
  int16_t wz_mrad_s;
} fw_velocity_command_t;

typedef struct {
  uint16_t sequence;
  int16_t vx_mm_s;
  int16_t vy_mm_s;
  int16_t wz_mrad_s;
  uint16_t battery_mv;
  bool estop;
  bool heartbeat_ok;
  fw_control_mode_t control_mode;
  bool obstacle_stop;
} fw_telemetry_t;

typedef struct {
  int16_t rpm[4];
} fw_wheel_feedback_t;

typedef struct {
  uint32_t command_timeout_ms;
  uint16_t nominal_battery_mv;
  int16_t pid_kp_milli;
  int16_t pid_ki_milli;
  int16_t pid_output_limit_permille;
  uint16_t ultrasonic_stop_mm;
} fw_runtime_config_t;

typedef struct {
  fw_velocity_command_t last_command;
  uint32_t last_command_ms;
  uint32_t last_step_ms;
  int32_t pid_integral[4];
} fw_runtime_state_t;

typedef struct {
  int16_t target_rpm[4];
  int16_t drive_permille[4];
  fw_telemetry_t telemetry;
} fw_runtime_result_t;

void fw_runtime_init(fw_runtime_state_t *state);
void fw_runtime_apply_command(fw_runtime_state_t *state, const fw_velocity_command_t *cmd, uint32_t now_ms);
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
    fw_runtime_result_t *out_result);

bool fw_parse_legacy_frame(const uint8_t *frame, size_t len, fw_velocity_command_t *out_cmd);
size_t fw_encode_managed_telemetry(const fw_telemetry_t *telemetry, uint8_t *out_frame, size_t out_capacity);

#ifdef __cplusplus
}
#endif
