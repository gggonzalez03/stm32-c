#include <stdint.h>
#include <stdio.h>
#include "stm32f411xe.h"

static void delay(uint32_t ms) {
  while(ms--);
}

static void config_port_c_pin_13() {
  uint32_t port_c_enable = (1UL << 2);
  uint32_t pin_13_output = (1UL << 26);
  RCC->AHB1ENR |= port_c_enable;
  GPIOC->MODER &= ~(3UL << 26);
  GPIOC->MODER |= pin_13_output;
}

int main() {
  config_port_c_pin_13();

  while (1) {
    delay(1000000);
    fprintf(stderr, "Hello World");
  }
  return 0;
}