#include "flash.h"
#include "stm32f411xe.h"


void flash__enable_prefetch() {
  FLASH->ACR |= (1UL << 8);
}

void flash__set_latency(uint8_t latency) {
  FLASH->ACR &= ~(0xF);
  FLASH->ACR |= (latency << 0);
}

uint8_t flash__get_latency() {
  return (uint8_t)((FLASH->ACR >> 0) & 0x0F);
}

void flash__config_3v_frq_64_90MHz() {
  flash__set_latency(2UL);
  while (flash__get_latency() != 2UL);
  flash__enable_prefetch();
}