#include <stdint.h>

uint32_t SystemCoreClock = 8000000U;

void SystemInit(void) {
  /*
   * Keep startup deterministic and self-contained:
   * the runtime assumes the default internal 8 MHz clock so UART / PWM /
   * SysTick divisors stay reproducible across build environments. Board-level
   * performance tuning can raise the clock later without changing the firmware
   * contract because every peripheral divisor derives from SystemCoreClock.
   */
  SystemCoreClock = 8000000U;
}

void SystemCoreClockUpdate(void) {
  SystemCoreClock = 8000000U;
}
