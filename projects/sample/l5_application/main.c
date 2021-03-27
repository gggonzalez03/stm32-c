#include <stdint.h>
#include <stdio.h>

#include "stm32f411xe.h"

#include "gpio.h"

static void delay(uint32_t ms) {
  while(ms--);
}

int main() {
  gpio__port_clock_enable(GPIO__PORT_C);
  gpio__gpio_s gpio = gpio__configure_as_output(GPIO__PORT_C, 13);

  while (1) {
    gpio__set(gpio);
    delay(100000);
    gpio__reset(gpio);
    delay(100000);
  }
  return 0;
}