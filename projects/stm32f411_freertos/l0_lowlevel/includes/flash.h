#pragma once

#include <stdint.h>

void flash__enable_prefetch();
void flash__set_latency(uint8_t latency);
uint8_t flash__get_latency();

/**
 * Flash configuration assuming that the voltage range is from
 * 2.7V to 3.6V and that the CPU clock frequency is between
 * 64MHz and 90MHz
 * NOTE: See Chapter 3.4.1
 **/
void flash__config_3v_frq_64_90MHz();