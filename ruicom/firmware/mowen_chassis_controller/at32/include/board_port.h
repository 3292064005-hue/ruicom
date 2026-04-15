#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "at32_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Authoritative AT32F403AVCT7 board contract for the MO-SERGEANT chassis.
 *
 * Hardware assumptions used by this firmware target:
 * - USART3 on PB10/PB11 for upper-computer serial bridge
 * - TIM8 CH1..CH4 on PC6..PC9 for four motor PWM channels
 * - PB12..PB15 as motor direction GPIOs
 * - TIM1/TIM2/TIM3/TIM4 configured in encoder mode for wheel feedback
 * - ADC1 channel 15 (PC5) samples battery divider voltage
 * - PC13 is an active-low emergency-stop input
 * - USART2 on PA2/PA3 receives voice-module text commands
 * - UART4 on PC10/PC11 receives ultrasonic aggregator text frames
 * - PS2 receiver is bit-banged on GPIOC[0..3] (ATT,CMD,CLK,DATA)
 */

enum {
  BOARD_WHEEL_COUNT = 4,
  BOARD_ENCODER_COUNTS_PER_REV = 550,
  BOARD_WHEEL_RADIUS_MM = 50,
  BOARD_HALF_SPAN_MM = 140,
  BOARD_UART_BAUDRATE = 115200,
  BOARD_PWM_FREQUENCY_HZ = 20000,
  BOARD_PWM_PERIOD_COUNTS = 400,
  BOARD_BATTERY_ADC_CHANNEL = 15,
  BOARD_BATTERY_DIVIDER_NUM = 3,
  BOARD_BATTERY_DIVIDER_DEN = 1,
  BOARD_VOICE_BAUDRATE = 115200,
  BOARD_ULTRASONIC_BAUDRATE = 115200,
};

void board_port_init(void);
uint32_t board_port_millis(void);
bool board_port_read_uart_byte(uint8_t *out_byte);
void board_port_write_uart(const uint8_t *data, size_t len);
void board_port_set_motor_pwm(uint8_t wheel_index, int16_t duty_permille);
void board_port_read_wheel_feedback(fw_wheel_feedback_t *out_feedback);
uint16_t board_port_read_battery_mv(void);
bool board_port_read_estop(void);
void board_port_watchdog_kick(void);

bool board_port_read_voice_byte(uint8_t *out_byte);
bool board_port_read_ultrasonic_byte(uint8_t *out_byte);
bool board_port_poll_ps2_packet(uint8_t out_packet[6]);

#ifdef __cplusplus
}
#endif
