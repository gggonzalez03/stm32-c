#include "wwdg.h"
#include "stm_peripherals.h"
#include "clock.h"

#include "stm32f411xe.h"

static uint8_t watchdog_downcounter_reload;

static uint8_t wwdg__get_7_bit_correlation(float ms, float divider)
{
  // This formula can be found in Chapter 16.4
  float apb1_clock_period = 1.0f / (float)(clock__get_apb1_clock_frq() / 1000);
  return (uint8_t)(ms / (apb1_clock_period * 4096.0f * divider)) - 1;
}

void wwdg__power_on(void)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_WWDG, false);
}

/**
 * NOTE:
 * 1. The t6_bit needs to be set to prevent an immediate restart
 *    (See Chapter 16.3 under the section "Controlling the downcounter")
 **/
void wwdg__enable(void)
{
  uint32_t activation_bit = (1 << 7);
  uint32_t t6_bit = (1 << 6);

  WWDG->CR = (watchdog_downcounter_reload | activation_bit | t6_bit);
}

/**
 * TODO:
 * 1. Calculate the timer base divider based on the desired max timeout
 * 2. Limit the downcounter reload value from 0x00 to 0x3F
 **/
void wwdg__set_max_timeout(float ms)
{
  uint32_t timer_base_div_8 = (3 << 7);

  watchdog_downcounter_reload = wwdg__get_7_bit_correlation(ms, 8.0f);

  WWDG->CFR |= timer_base_div_8;
}

void wwdg__set_window_start(float ms)
{
  uint32_t window_start_mask = (0x7F << 0);

  uint8_t window_start = (uint8_t)window_start_mask - wwdg__get_7_bit_correlation(ms, 8.0f);

  WWDG->CFR &= ~(window_start_mask);
  WWDG->CFR |= window_start;
}

void wwdg__check_in()
{
  uint32_t t6_bit = (1 << 6);
  uint32_t activation_bit = (1 << 7);

  WWDG->CR = (watchdog_downcounter_reload | activation_bit | t6_bit) & 0xFF;
}