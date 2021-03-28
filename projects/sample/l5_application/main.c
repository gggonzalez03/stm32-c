#include <stdint.h>
#include <stdio.h>

#include "stm32f411xe.h"

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
  TIM2->PSC = 41;
  TIM2->CR1 |= start_counter;
  TIM2->EGR |= generate_update_event; // Required for the prescaler value to take effect
}

int main() {
  gpio__port_clock_enable(GPIO__PORT_C);
  gpio__gpio_s gpio_0 = gpio__configure_as_output(GPIO__PORT_C, 13);
  gpio__gpio_s gpio_1 = gpio__configure_as_output(GPIO__PORT_C, 15);

  timer__enable_counter();

  while (1) {
    gpio__toggle(gpio_1);
    crude_delay_ms(1000);
  }
  return 0;
}