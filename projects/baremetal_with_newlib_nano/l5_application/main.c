#include <stdint.h>
#include <stdio.h>

#include "stm32f411xe.h"

#include "clock.h"
#include "gpio.h"

static void crude_delay_ms(uint32_t ms) {
  uint32_t count_up_to = TIM2->CNT + (ms * 1000);
  while (TIM2->CNT < count_up_to);
}

static void timer__enable_counter()
{
  uint32_t timer_2_enable = (1UL << 0);
  uint32_t start_counter = (1UL << 0);
  uint32_t generate_update_event = (1UL << 0);

  RCC->APB1ENR |= timer_2_enable;
  /**
   * Clocks on timers on APB1 bus are multiplied by 2 if the APB1 bus
   * prescaler is greater than 1. So basically the minimum frequency of
   * timer clocks is the AHB bus frequency
   * See Chapter 6.2, Figure 12.
   **/ 
  TIM2->PSC = (clock__get_core_clock_frq() / (1000UL * 1000UL) - 1UL);
  TIM2->CR1 |= start_counter;
  TIM2->EGR |= generate_update_event; // Required for the prescaler value to take effect
}

int main()
{
  gpio__port_clock_enable(GPIO__PORT_C);
  gpio__gpio_s gpio_0 = gpio__configure_as_output(GPIO__PORT_C, 15);

  timer__enable_counter();

  while (1) {
    gpio__toggle(gpio_0);
    crude_delay_ms(500);
  }
  return 0;
}