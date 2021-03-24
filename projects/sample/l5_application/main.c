#include <stdint.h>
#include <stdio.h>
#include "stm32f411xe.h"

static void delay(uint32_t ms) {
  while(ms--);
}

static void startup__test_gpio_a() {
  uint32_t port_a_enable = (1UL << 0);
  uint32_t pin_0_output = (1UL << 0);
  RCC->AHB1ENR |= port_a_enable;
  GPIOA->MODER &= ~(3UL << 0);
  GPIOA->MODER |= pin_0_output;
}

int main() {
  startup__test_gpio_a();
  delay(1000000);
  fprintf(stderr, "ads");
  delay(1000000);
  fprintf(stderr, "ads");
  delay(1000000);
  fprintf(stderr, "ads");
  delay(1000000);
  fprintf(stderr, "ads");
  return 0;
}