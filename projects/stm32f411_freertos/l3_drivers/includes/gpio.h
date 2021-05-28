#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @note Depending on the chip package, some of these port may
 * not be available
 **/
typedef enum
{
  GPIO__PORT_A = 0,
  GPIO__PORT_B,
  GPIO__PORT_C,
  GPIO__PORT_D,
  GPIO__PORT_E,
  GPIO__PORT_H = 7,
} gpio__port_e;

/**
 * @note These set frequencies assume that the VDD >= 2.7V
 * Refer to page 101 of the datasheet (Chapter 6.3.16)
 **/
typedef enum
{
  GPIO__8MHz = 0,
  GPIO__50MHz
} gpio__speed_e;

/**
 * @note Check out the datasheet from page 47 to see what these 
 * alternate function enums correspond to for specific pins. There should
 * be a table that says "Alternate function mapping"
 **/
typedef enum
{
  GPIO__AF00 = 0,
  GPIO__AF01,
  GPIO__AF02,
  GPIO__AF03,
  GPIO__AF04,
  GPIO__AF05,
  GPIO__AF06,
  GPIO__AF07,
  GPIO__AF08,
  GPIO__AF09,
  GPIO__AF10,
  GPIO__AF11,
  GPIO__AF12,
  GPIO__AF13,
  GPIO__AF14,
  GPIO__AF15,
} gpio__alternate_function_e;

typedef enum
{
  GPIO__FALLING_EDGE = 0,
  GPIO__RISING_EDGE
} gpio__interrupt_edge_e;

typedef struct
{
  gpio__port_e port;
  uint8_t pin;
} gpio__gpio_s;

void gpio__port_clock_enable(gpio__port_e port);

gpio__gpio_s gpio__construct(gpio__port_e port, uint8_t pin);
gpio__gpio_s gpio__configure_as_input(gpio__port_e port, uint8_t pin);
gpio__gpio_s gpio__configure_as_output(gpio__port_e port, uint8_t pin);
gpio__gpio_s gpio__configure_with_function(gpio__port_e port, uint8_t pin, gpio__alternate_function_e function);

void gpio__configure_speed(gpio__gpio_s gpio, gpio__speed_e speed);

void gpio__set_function(gpio__gpio_s gpio, gpio__alternate_function_e function);
void gpio__set_as_input(gpio__gpio_s gpio);
void gpio__set_as_output(gpio__gpio_s gpio);
void gpio__set_as_alternate_function(gpio__gpio_s gpio);

void gpio__enable_pull_up_resistor(gpio__gpio_s gpio);
void gpio__enable_pull_down_resistor(gpio__gpio_s gpio);

void gpio__set(gpio__gpio_s gpio);
void gpio__reset(gpio__gpio_s gpio);
bool gpio__get(gpio__gpio_s gpio);
void gpio__toggle(gpio__gpio_s gpio);

void gpio__enable_interrupt(gpio__gpio_s gpio, gpio__interrupt_edge_e edge);