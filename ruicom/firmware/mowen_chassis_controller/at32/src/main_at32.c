#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "at32_runtime.h"
#include "board_port.h"

static bool poll_command(fw_velocity_command_t *out_cmd) {
  uint8_t frame[12];
  static uint8_t window[12];
  static uint8_t count = 0U;
  uint8_t byte = 0U;
  while (board_port_read_uart_byte(&byte)) {
    if (count < 12U) {
      window[count++] = byte;
    } else {
      for (uint8_t idx = 1U; idx < 12U; ++idx) {
        window[idx - 1U] = window[idx];
      }
      window[11] = byte;
    }
    if (count >= 12U) {
      for (uint8_t idx = 0U; idx < 12U; ++idx) frame[idx] = window[idx];
      if (fw_parse_legacy_frame(frame, 12U, out_cmd)) {
        return true;
      }
    }
  }
  return false;
}

static bool streq(const char *lhs, const char *rhs) {
  size_t idx = 0U;
  while (lhs[idx] != '\0' && rhs[idx] != '\0') {
    if (lhs[idx] != rhs[idx]) return false;
    ++idx;
  }
  return lhs[idx] == rhs[idx];
}

static bool parse_uint16(const char *text, uint16_t *out_value) {
  if (text == 0 || out_value == 0 || *text == '\0') return false;
  uint32_t value = 0U;
  while (*text != '\0') {
    if (*text < '0' || *text > '9') return false;
    value = value * 10U + (uint32_t)(*text - '0');
    if (value > 65535U) return false;
    ++text;
  }
  *out_value = (uint16_t)value;
  return true;
}

static bool poll_voice_line(char *out_line, size_t capacity) {
  static char buffer[64];
  static size_t used = 0U;
  uint8_t byte = 0U;
  while (board_port_read_voice_byte(&byte)) {
    if (byte == '\r') continue;
    if (byte == '\n') {
      if (used == 0U) continue;
      size_t copy_len = used < (capacity - 1U) ? used : (capacity - 1U);
      for (size_t idx = 0U; idx < copy_len; ++idx) out_line[idx] = buffer[idx];
      out_line[copy_len] = '\0';
      used = 0U;
      return true;
    }
    if (used + 1U < sizeof(buffer)) {
      buffer[used++] = (char)byte;
    } else {
      used = 0U;
    }
  }
  return false;
}

static bool poll_ultrasonic_frame(uint16_t ranges_mm[4]) {
  static char buffer[64];
  static size_t used = 0U;
  uint8_t byte = 0U;
  while (board_port_read_ultrasonic_byte(&byte)) {
    if (byte == '\r') continue;
    if (byte == '\n') {
      if (used == 0U) continue;
      buffer[used] = '\0';
      used = 0U;
      if (buffer[0] != 'U' || buffer[1] != ',') return false;
      char *cursor = &buffer[2];
      for (uint8_t idx = 0U; idx < 4U; ++idx) {
        char token[8];
        size_t token_len = 0U;
        while (*cursor != '\0' && *cursor != ',' && token_len + 1U < sizeof(token)) {
          token[token_len++] = *cursor++;
        }
        token[token_len] = '\0';
        if (!parse_uint16(token, &ranges_mm[idx])) return false;
        if ((idx < 3U && *cursor != ',') || (idx == 3U && *cursor != '\0')) return false;
        if (*cursor == ',') ++cursor;
      }
      return true;
    }
    if (used + 1U < sizeof(buffer)) {
      buffer[used++] = (char)byte;
    } else {
      used = 0U;
    }
  }
  return false;
}

static bool parse_voice_command(const char *line, fw_control_mode_t *mode, fw_velocity_command_t *voice_cmd) {
  if (streq(line, "COMMANDER")) {
    *mode = FW_CONTROL_MODE_COMMANDER;
    return false;
  }
  if (streq(line, "HANDLE")) {
    *mode = FW_CONTROL_MODE_HANDLE;
    return false;
  }
  if (streq(line, "VOICE")) {
    *mode = FW_CONTROL_MODE_VOICE;
    return false;
  }
  if (*mode != FW_CONTROL_MODE_VOICE) return false;
  if (streq(line, "STOP")) {
    voice_cmd->vx_mm_s = 0;
    voice_cmd->vy_mm_s = 0;
    voice_cmd->wz_mrad_s = 0;
    return true;
  }
  if (streq(line, "FORWARD")) {
    voice_cmd->vx_mm_s = 250;
    voice_cmd->vy_mm_s = 0;
    voice_cmd->wz_mrad_s = 0;
    return true;
  }
  if (streq(line, "BACKWARD")) {
    voice_cmd->vx_mm_s = -250;
    voice_cmd->vy_mm_s = 0;
    voice_cmd->wz_mrad_s = 0;
    return true;
  }
  if (streq(line, "LEFT")) {
    voice_cmd->vx_mm_s = 0;
    voice_cmd->vy_mm_s = 250;
    voice_cmd->wz_mrad_s = 0;
    return true;
  }
  if (streq(line, "RIGHT")) {
    voice_cmd->vx_mm_s = 0;
    voice_cmd->vy_mm_s = -250;
    voice_cmd->wz_mrad_s = 0;
    return true;
  }
  if (streq(line, "ROTATE_LEFT")) {
    voice_cmd->vx_mm_s = 0;
    voice_cmd->vy_mm_s = 0;
    voice_cmd->wz_mrad_s = 700;
    return true;
  }
  if (streq(line, "ROTATE_RIGHT")) {
    voice_cmd->vx_mm_s = 0;
    voice_cmd->vy_mm_s = 0;
    voice_cmd->wz_mrad_s = -700;
    return true;
  }
  return false;
}

static int16_t ps2_axis_to_permille(uint8_t value) {
  int16_t centered = (int16_t)value - 128;
  if (centered > -12 && centered < 12) return 0;
  int32_t permille = ((int32_t)centered * 1000) / 127;
  if (permille > 1000) permille = 1000;
  if (permille < -1000) permille = -1000;
  return (int16_t)permille;
}

static bool poll_ps2_command(fw_control_mode_t *mode, fw_velocity_command_t *ps2_cmd) {
  uint8_t packet[6];
  if (!board_port_poll_ps2_packet(packet)) return false;
  /* Typical packet: [0xFF][0x5A][buttons_low][buttons_high][rx][ry] */
  uint16_t buttons = (uint16_t)packet[2] | ((uint16_t)packet[3] << 8U);
  if ((buttons & (1U << 0)) == 0U) *mode = FW_CONTROL_MODE_COMMANDER;
  if ((buttons & (1U << 1)) == 0U) *mode = FW_CONTROL_MODE_HANDLE;
  if ((buttons & (1U << 2)) == 0U) *mode = FW_CONTROL_MODE_VOICE;
  if (*mode != FW_CONTROL_MODE_HANDLE) return false;
  int16_t vx_permille = (int16_t)(-ps2_axis_to_permille(packet[5]));
  int16_t vy_permille = (int16_t)(ps2_axis_to_permille(packet[4]));
  int16_t wz_permille = 0;
  if ((buttons & (1U << 7)) == 0U) wz_permille = 450;
  if ((buttons & (1U << 6)) == 0U) wz_permille = -450;
  ps2_cmd->vx_mm_s = (int16_t)((vx_permille * 400) / 1000);
  ps2_cmd->vy_mm_s = (int16_t)((vy_permille * 300) / 1000);
  ps2_cmd->wz_mrad_s = (int16_t)((wz_permille * 1000) / 1000);
  return true;
}

static bool ultrasonic_obstacle_stop(const uint16_t ranges_mm[4], uint16_t threshold_mm) {
  for (uint8_t idx = 0U; idx < 4U; ++idx) {
    if (ranges_mm[idx] != 0U && ranges_mm[idx] < threshold_mm) return true;
  }
  return false;
}

int main(void) {
  fw_runtime_state_t state;
  fw_runtime_config_t config = {500U, 11800U, 350, 20, 1000, 250U};
  fw_runtime_result_t result;
  fw_wheel_feedback_t feedback;
  fw_velocity_command_t serial_command;
  fw_velocity_command_t aux_command = {0, 0, 0};
  fw_control_mode_t control_mode = FW_CONTROL_MODE_COMMANDER;
  uint16_t ultrasonic_ranges[4] = {0U, 0U, 0U, 0U};
  uint16_t sequence = 0U;
  uint8_t telemetry_frame[32];
  char voice_line[64];

  board_port_init();
  fw_runtime_init(&state);

  for (;;) {
    uint32_t now_ms = board_port_millis();
    if (poll_command(&serial_command) && control_mode == FW_CONTROL_MODE_COMMANDER) {
      fw_runtime_apply_command(&state, &serial_command, now_ms);
    }
    if (poll_voice_line(voice_line, sizeof(voice_line)) && parse_voice_command(voice_line, &control_mode, &aux_command)) {
      fw_runtime_apply_command(&state, &aux_command, now_ms);
    }
    if (poll_ps2_command(&control_mode, &aux_command)) {
      fw_runtime_apply_command(&state, &aux_command, now_ms);
    }
    if (poll_ultrasonic_frame(ultrasonic_ranges)) {
      /* latest readings latched in ultrasonic_ranges */
    }
    bool soft_stop = ultrasonic_obstacle_stop(ultrasonic_ranges, config.ultrasonic_stop_mm);
    board_port_read_wheel_feedback(&feedback);
    fw_runtime_step(
        &state,
        &config,
        &feedback,
        board_port_read_battery_mv(),
        (bool)(board_port_read_estop() || soft_stop),
        control_mode,
        soft_stop,
        now_ms,
        ++sequence,
        &result);
    for (uint8_t idx = 0U; idx < 4U; ++idx) {
      board_port_set_motor_pwm(idx, result.drive_permille[idx]);
    }
    size_t frame_len = fw_encode_managed_telemetry(&result.telemetry, telemetry_frame, sizeof(telemetry_frame));
    board_port_write_uart(telemetry_frame, frame_len);
    board_port_watchdog_kick();
  }
}
