#pragma once

#include <stdint.h>

typedef enum
{
  EXTI__GPIO_PORT_A = 0,
  EXTI__GPIO_PORT_B,
  EXTI__GPIO_PORT_C,
  EXTI__GPIO_PORT_D,
  EXTI__GPIO_PORT_E,
  EXTI__GPIO_PORT_H = 7,
} exti__gpio_port_e;

typedef enum
{
  EXTI__FALLING_EDGE = 0,
  EXTI__RISING_EDGE
} exti__trigger_edge_e;

/**
 * Select a source for an interrupt line.
 * @param exti_number external interrupt line to select source for. Value from 0 to 15.
 * @param source value from 0 to 4 and 7 to select the source. Refer to Chapter 7.2 of the Reference Manual.
 **/
void exti__select_interrupt_line_source(uint8_t exti_number, uint8_t source);

/**
 * Setup external interrupt.
 * @param exti_number external interrupt line to setup. Value from 0 to 18, 21, and 22.
 * @param edge edge trigger, i.e., rising or falling.
 **/
void exti__setup_external_interrupt(uint8_t exti_number, exti__trigger_edge_e edge);

/**
 * Enable external interrupt.
 * @param exti_numbeer external interrupt line to enable. Value from 0 to 18, 21, and 22.
 **/
void exti__enable_external_interrupt(uint8_t exti_number);

/**
 * Clear the pending bit of an interrupt
 * @param exti_number external interrupt line to clear
 **/
void exti__clear_external_interrupt(uint8_t exti_number);