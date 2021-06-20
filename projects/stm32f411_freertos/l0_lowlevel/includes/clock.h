#pragma once

#include <stdint.h>

/**
 * Initialize system clock
 **/ 
void clock__init_system_clock();

/**
 * Get the core clock in Hertz
 * @return uint32_t clock frequency in Hertz
 **/
uint32_t clock__get_core_clock_frq();

/**
 * Get the clock on the APB1 bus in Hertz
 **/ 
uint32_t clock__get_apb1_clock_frq(void);