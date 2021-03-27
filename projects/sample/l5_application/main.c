#include <stdint.h>
#include <stdio.h>

#include "stm32f411xe.h"

#include "gpio.h"

static void delay(uint32_t ms) {
  while(ms--);
}

int main() {
  gpio__port_clock_enable(GPIO__PORT_C);
  gpio__gpio_s gpio_0 = gpio__configure_as_output(GPIO__PORT_C, 13);
  gpio__gpio_s gpio_1 = gpio__configure_as_output(GPIO__PORT_C, 15);

  while (1) {
    gpio__set(gpio_1);
    gpio__reset(gpio_0);
    delay(1000000);
    gpio__set(gpio_0);
    gpio__reset(gpio_1);
    delay(1000000);
  }
  return 0;
}