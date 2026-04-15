#include "board_port.h"

#include <stddef.h>
#include <stdint.h>

extern uint32_t SystemCoreClock;

#define REG32(address) (*(volatile uint32_t *)(address))

#define SYSTICK_CTRL REG32(0xE000E010UL)
#define SYSTICK_LOAD REG32(0xE000E014UL)
#define SYSTICK_VAL  REG32(0xE000E018UL)

#define NVIC_ICER0 REG32(0xE000E180UL)
#define NVIC_ICPR0 REG32(0xE000E280UL)

#define USART_SR_RXNE (1U << 5)
#define USART_SR_TC   (1U << 6)
#define USART_SR_TXE  (1U << 7)
#define USART_CR1_RE  (1U << 2)
#define USART_CR1_TE  (1U << 3)
#define USART_CR1_UE  (1U << 13)

#define TIM_CR1_CEN   (1U << 0)
#define TIM_CR1_ARPE  (1U << 7)
#define TIM_SMCR_SMS_ENCODER3 0x3U
#define TIM_CCER_CC1E (1U << 0)
#define TIM_CCER_CC2E (1U << 4)
#define TIM_CCER_CC3E (1U << 8)
#define TIM_CCER_CC4E (1U << 12)
#define TIM_CCMR_OC_PWM1 0x6U
#define TIM_CCMR_OC_PRELOAD (1U << 3)
#define TIM_BDTR_MOE (1U << 15)
#define TIM_EGR_UG (1U << 0)

#define ADC_SR_EOC     (1U << 1)
#define ADC_CR2_ADON   (1U << 0)
#define ADC_CR2_CAL    (1U << 2)
#define ADC_CR2_RSTCAL (1U << 3)
#define ADC_CR2_EXTTRIG (1U << 20)
#define ADC_CR2_SWSTART (1U << 22)

#define IWDG_KR_ENABLE 0xCCCCU
#define IWDG_KR_RELOAD 0xAAAAU
#define IWDG_KR_UNLOCK 0x5555U

typedef struct {
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  volatile uint32_t MAPR2;
} afio_registers_t;

typedef struct {
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} gpio_registers_t;

typedef struct {
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  volatile uint32_t AHBRSTR;
  volatile uint32_t CFGR2;
  volatile uint32_t CFGR3;
} rcc_registers_t;

typedef struct {
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t BRR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t GTPR;
} usart_registers_t;

typedef struct {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMCR;
  volatile uint32_t DIER;
  volatile uint32_t SR;
  volatile uint32_t EGR;
  volatile uint32_t CCMR1;
  volatile uint32_t CCMR2;
  volatile uint32_t CCER;
  volatile uint32_t CNT;
  volatile uint32_t PSC;
  volatile uint32_t ARR;
  volatile uint32_t RCR;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint32_t BDTR;
  volatile uint32_t DCR;
  volatile uint32_t DMAR;
} tim_registers_t;

typedef struct {
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} adc_registers_t;

typedef struct {
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} iwdg_registers_t;

#define AFIO   ((afio_registers_t *)0x40010000UL)
#define GPIOA  ((gpio_registers_t *)0x40010800UL)
#define GPIOB  ((gpio_registers_t *)0x40010C00UL)
#define GPIOC  ((gpio_registers_t *)0x40011000UL)
#define RCC    ((rcc_registers_t  *)0x40021000UL)
#define USART1 ((usart_registers_t *)0x40013800UL)
#define USART2 ((usart_registers_t *)0x40004400UL)
#define USART3 ((usart_registers_t *)0x40004800UL)
#define UART4  ((usart_registers_t *)0x40004C00UL)
#define TIM1   ((tim_registers_t *)0x40012C00UL)
#define TIM2   ((tim_registers_t *)0x40000000UL)
#define TIM3   ((tim_registers_t *)0x40000400UL)
#define TIM4   ((tim_registers_t *)0x40000800UL)
#define TIM8   ((tim_registers_t *)0x40013400UL)
#define ADC1   ((adc_registers_t *)0x40012400UL)
#define IWDG   ((iwdg_registers_t *)0x40003000UL)

static volatile uint32_t g_millis = 0U;
static uint16_t g_encoder_prev_count[BOARD_WHEEL_COUNT] = {0U, 0U, 0U, 0U};
static int16_t g_encoder_last_rpm[BOARD_WHEEL_COUNT] = {0, 0, 0, 0};
static uint32_t g_encoder_prev_ms = 0U;

static tim_registers_t *const k_encoder_timers[BOARD_WHEEL_COUNT] = {TIM1, TIM2, TIM3, TIM4};
static volatile uint32_t *const k_pwm_compare[BOARD_WHEEL_COUNT] = {&TIM8->CCR1, &TIM8->CCR2, &TIM8->CCR3, &TIM8->CCR4};
static gpio_registers_t *const k_dir_ports[BOARD_WHEEL_COUNT] = {GPIOB, GPIOB, GPIOB, GPIOB};
static const uint8_t k_dir_pins[BOARD_WHEEL_COUNT] = {12U, 13U, 14U, 15U};

/* Bit-banged PS2 pins on GPIOC. */
static const uint8_t k_ps2_att_pin = 0U;
static const uint8_t k_ps2_cmd_pin = 1U;
static const uint8_t k_ps2_clk_pin = 2U;
static const uint8_t k_ps2_dat_pin = 3U;

void SysTick_Handler(void) {
  g_millis += 1U;
}

static void gpio_configure(gpio_registers_t *gpio, uint8_t pin, uint8_t mode_bits) {
  volatile uint32_t *cfg = pin < 8U ? &gpio->CRL : &gpio->CRH;
  uint8_t shift = (uint8_t)((pin & 0x7U) * 4U);
  uint32_t value = *cfg;
  value &= ~(0xFU << shift);
  value |= ((uint32_t)mode_bits & 0xFU) << shift;
  *cfg = value;
}

static void gpio_write(gpio_registers_t *gpio, uint8_t pin, bool high) {
  if (high) {
    gpio->BSRR = 1UL << pin;
  } else {
    gpio->BRR = 1UL << pin;
  }
}

static bool gpio_read(const gpio_registers_t *gpio, uint8_t pin) {
  return ((gpio->IDR >> pin) & 0x1U) != 0U;
}

static void busy_delay_cycles(uint32_t cycles) {
  while (cycles-- != 0U) {
    __asm__ volatile("nop");
  }
}

static void systick_init(void) {
  uint32_t reload = (SystemCoreClock / 1000U);
  if (reload == 0U) reload = 1U;
  SYSTICK_LOAD = reload - 1U;
  SYSTICK_VAL = 0U;
  SYSTICK_CTRL = 0x07U;
}

static void uart_configure(usart_registers_t *uart, uint32_t baudrate) {
  uart->CR1 = 0U;
  uart->CR2 = 0U;
  uart->CR3 = 0U;
  uart->BRR = (SystemCoreClock + (baudrate / 2U)) / baudrate;
  uart->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}

static void uart_init(void) {
  /* PB10 TX alternate push-pull, PB11 RX floating input. */
  gpio_configure(GPIOB, 10U, 0xBU);
  gpio_configure(GPIOB, 11U, 0x4U);
  uart_configure(USART3, BOARD_UART_BAUDRATE);
}

static void aux_uart_init(void) {
  /* Voice UART on USART2 PA2/PA3. */
  gpio_configure(GPIOA, 2U, 0xBU);
  gpio_configure(GPIOA, 3U, 0x4U);
  uart_configure(USART2, BOARD_VOICE_BAUDRATE);

  /* Ultrasonic UART on UART4 PC10/PC11. */
  gpio_configure(GPIOC, 10U, 0xBU);
  gpio_configure(GPIOC, 11U, 0x4U);
  uart_configure(UART4, BOARD_ULTRASONIC_BAUDRATE);
}

static void pwm_init(void) {
  gpio_configure(GPIOC, 6U, 0xBU);
  gpio_configure(GPIOC, 7U, 0xBU);
  gpio_configure(GPIOC, 8U, 0xBU);
  gpio_configure(GPIOC, 9U, 0xBU);
  for (uint8_t idx = 0U; idx < BOARD_WHEEL_COUNT; ++idx) {
    gpio_configure(k_dir_ports[idx], k_dir_pins[idx], 0x2U);
    gpio_write(k_dir_ports[idx], k_dir_pins[idx], false);
  }

  TIM8->CR1 = 0U;
  TIM8->PSC = 0U;
  TIM8->ARR = BOARD_PWM_PERIOD_COUNTS - 1U;
  TIM8->CCR1 = 0U;
  TIM8->CCR2 = 0U;
  TIM8->CCR3 = 0U;
  TIM8->CCR4 = 0U;
  TIM8->CCMR1 = (TIM_CCMR_OC_PWM1 << 4) | TIM_CCMR_OC_PRELOAD |
                (TIM_CCMR_OC_PWM1 << 12) | (TIM_CCMR_OC_PRELOAD << 8);
  TIM8->CCMR2 = (TIM_CCMR_OC_PWM1 << 4) | TIM_CCMR_OC_PRELOAD |
                (TIM_CCMR_OC_PWM1 << 12) | (TIM_CCMR_OC_PRELOAD << 8);
  TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM8->BDTR = TIM_BDTR_MOE;
  TIM8->EGR = TIM_EGR_UG;
  TIM8->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void encoder_timer_init(tim_registers_t *timer) {
  timer->CR1 = 0U;
  timer->PSC = 0U;
  timer->ARR = 0xFFFFU;
  timer->CCMR1 = 0x0101U;
  timer->CCER = 0U;
  timer->SMCR = TIM_SMCR_SMS_ENCODER3;
  timer->CNT = 0U;
  timer->EGR = TIM_EGR_UG;
  timer->CR1 = TIM_CR1_CEN;
}

static void encoder_init(void) {
  /* Wheel0: TIM1 PA8/PA9, Wheel1: TIM2 PA0/PA1, Wheel2: TIM3 PA6/PA7, Wheel3: TIM4 PB6/PB7 */
  gpio_configure(GPIOA, 8U, 0x4U);
  gpio_configure(GPIOA, 9U, 0x4U);
  gpio_configure(GPIOA, 0U, 0x4U);
  gpio_configure(GPIOA, 1U, 0x4U);
  gpio_configure(GPIOA, 6U, 0x4U);
  gpio_configure(GPIOA, 7U, 0x4U);
  gpio_configure(GPIOB, 6U, 0x4U);
  gpio_configure(GPIOB, 7U, 0x4U);

  encoder_timer_init(TIM1);
  encoder_timer_init(TIM2);
  encoder_timer_init(TIM3);
  encoder_timer_init(TIM4);
}

static void adc_init(void) {
  gpio_configure(GPIOC, 5U, 0x0U);
  ADC1->CR1 = 0U;
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SMPR1 = 0x7U << 15; /* channel 15, longest sampling */
  ADC1->SQR1 = 0U;
  ADC1->SQR2 = 0U;
  ADC1->SQR3 = BOARD_BATTERY_ADC_CHANNEL;
  ADC1->CR2 |= ADC_CR2_RSTCAL;
  while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U) {
  }
  ADC1->CR2 |= ADC_CR2_CAL;
  while ((ADC1->CR2 & ADC_CR2_CAL) != 0U) {
  }
}

static void estop_init(void) {
  gpio_configure(GPIOC, 13U, 0x8U); /* input pull-up / pull-down */
  gpio_write(GPIOC, 13U, true);
}

static void watchdog_init(void) {
  IWDG->KR = IWDG_KR_UNLOCK;
  IWDG->PR = 0x4U;
  IWDG->RLR = 1000U;
  IWDG->KR = IWDG_KR_RELOAD;
  IWDG->KR = IWDG_KR_ENABLE;
}

static void ps2_init(void) {
  gpio_configure(GPIOC, k_ps2_att_pin, 0x2U);
  gpio_configure(GPIOC, k_ps2_cmd_pin, 0x2U);
  gpio_configure(GPIOC, k_ps2_clk_pin, 0x2U);
  gpio_configure(GPIOC, k_ps2_dat_pin, 0x4U);
  gpio_write(GPIOC, k_ps2_att_pin, true);
  gpio_write(GPIOC, k_ps2_cmd_pin, true);
  gpio_write(GPIOC, k_ps2_clk_pin, true);
}

void board_port_init(void) {
  NVIC_ICER0 = 0xFFFFFFFFUL;
  NVIC_ICPR0 = 0xFFFFFFFFUL;
  RCC->APB2ENR |= (1U << 0) | (1U << 2) | (1U << 3) | (1U << 4) | (1U << 9) | (1U << 11) | (1U << 13) | (1U << 14);
  RCC->APB1ENR |= (1U << 0) | (1U << 1) | (1U << 2) | (1U << 17) | (1U << 18) | (1U << 19);

  AFIO->MAPR = 0U;

  uart_init();
  aux_uart_init();
  pwm_init();
  encoder_init();
  adc_init();
  estop_init();
  watchdog_init();
  ps2_init();
  systick_init();

  for (uint8_t idx = 0U; idx < BOARD_WHEEL_COUNT; ++idx) {
    g_encoder_prev_count[idx] = (uint16_t)k_encoder_timers[idx]->CNT;
    g_encoder_last_rpm[idx] = 0;
  }
  g_encoder_prev_ms = 0U;
}

uint32_t board_port_millis(void) {
  return g_millis;
}

bool board_port_read_uart_byte(uint8_t *out_byte) {
  if (out_byte == 0) return false;
  if ((USART3->SR & USART_SR_RXNE) == 0U) return false;
  *out_byte = (uint8_t)(USART3->DR & 0xFFU);
  return true;
}

bool board_port_read_voice_byte(uint8_t *out_byte) {
  if (out_byte == 0) return false;
  if ((USART2->SR & USART_SR_RXNE) == 0U) return false;
  *out_byte = (uint8_t)(USART2->DR & 0xFFU);
  return true;
}

bool board_port_read_ultrasonic_byte(uint8_t *out_byte) {
  if (out_byte == 0) return false;
  if ((UART4->SR & USART_SR_RXNE) == 0U) return false;
  *out_byte = (uint8_t)(UART4->DR & 0xFFU);
  return true;
}

void board_port_write_uart(const uint8_t *data, size_t len) {
  if (data == 0U) return;
  for (size_t idx = 0U; idx < len; ++idx) {
    while ((USART3->SR & USART_SR_TXE) == 0U) {
    }
    USART3->DR = data[idx];
  }
  while ((USART3->SR & USART_SR_TC) == 0U) {
  }
}

void board_port_set_motor_pwm(uint8_t wheel_index, int16_t duty_permille) {
  if (wheel_index >= BOARD_WHEEL_COUNT) return;
  bool reverse = duty_permille < 0;
  uint16_t abs_permille = (uint16_t)(reverse ? -duty_permille : duty_permille);
  if (abs_permille > 1000U) abs_permille = 1000U;
  uint32_t compare = (uint32_t)abs_permille * (uint32_t)(BOARD_PWM_PERIOD_COUNTS - 1U) / 1000U;
  *k_pwm_compare[wheel_index] = compare;
  gpio_write(k_dir_ports[wheel_index], k_dir_pins[wheel_index], reverse);
}

void board_port_read_wheel_feedback(fw_wheel_feedback_t *out_feedback) {
  if (out_feedback == 0U) return;
  uint32_t now_ms = g_millis;
  uint32_t dt_ms = now_ms - g_encoder_prev_ms;
  if (dt_ms == 0U) {
    for (uint8_t idx = 0U; idx < BOARD_WHEEL_COUNT; ++idx) {
      out_feedback->rpm[idx] = g_encoder_last_rpm[idx];
    }
    return;
  }
  for (uint8_t idx = 0U; idx < BOARD_WHEEL_COUNT; ++idx) {
    uint16_t count = (uint16_t)k_encoder_timers[idx]->CNT;
    int16_t delta = (int16_t)(count - g_encoder_prev_count[idx]);
    int32_t rpm = ((int32_t)delta * 60000L) / ((int32_t)BOARD_ENCODER_COUNTS_PER_REV * (int32_t)dt_ms);
    if (rpm > 32767L) rpm = 32767L;
    if (rpm < -32768L) rpm = -32768L;
    g_encoder_prev_count[idx] = count;
    g_encoder_last_rpm[idx] = (int16_t)rpm;
    out_feedback->rpm[idx] = (int16_t)rpm;
  }
  g_encoder_prev_ms = now_ms;
}

uint16_t board_port_read_battery_mv(void) {
  ADC1->SR = 0U;
  ADC1->CR2 |= ADC_CR2_ADON;
  ADC1->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_SWSTART;
  for (uint32_t guard = 0U; guard < 1000000U; ++guard) {
    if ((ADC1->SR & ADC_SR_EOC) != 0U) {
      uint32_t raw = ADC1->DR & 0x0FFFU;
      uint32_t mv = raw * 3300UL * (uint32_t)BOARD_BATTERY_DIVIDER_NUM / (4095UL * (uint32_t)BOARD_BATTERY_DIVIDER_DEN);
      if (mv > 65535UL) mv = 65535UL;
      return (uint16_t)mv;
    }
  }
  return 0U;
}

bool board_port_read_estop(void) {
  return !gpio_read(GPIOC, 13U);
}

void board_port_watchdog_kick(void) {
  IWDG->KR = IWDG_KR_RELOAD;
}

static uint8_t ps2_shift_byte(uint8_t tx_byte) {
  uint8_t rx_byte = 0U;
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    gpio_write(GPIOC, k_ps2_cmd_pin, (tx_byte & (1U << bit)) != 0U);
    gpio_write(GPIOC, k_ps2_clk_pin, false);
    busy_delay_cycles(32U);
    if (gpio_read(GPIOC, k_ps2_dat_pin)) {
      rx_byte |= (uint8_t)(1U << bit);
    }
    gpio_write(GPIOC, k_ps2_clk_pin, true);
    busy_delay_cycles(32U);
  }
  return rx_byte;
}

bool board_port_poll_ps2_packet(uint8_t out_packet[6]) {
  if (out_packet == 0U) return false;
  static uint32_t last_poll_ms = 0U;
  uint32_t now_ms = g_millis;
  if ((now_ms - last_poll_ms) < 20U) {
    return false;
  }
  last_poll_ms = now_ms;
  const uint8_t tx[6] = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U};
  gpio_write(GPIOC, k_ps2_att_pin, false);
  busy_delay_cycles(64U);
  for (uint8_t idx = 0U; idx < 6U; ++idx) {
    out_packet[idx] = ps2_shift_byte(tx[idx]);
  }
  gpio_write(GPIOC, k_ps2_att_pin, true);
  return out_packet[1] != 0U;
}
