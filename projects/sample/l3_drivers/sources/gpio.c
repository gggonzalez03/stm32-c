#include <stdlib.h>

#include "gpio.h"
#include "stm32f411xe.h"

static const GPIO_TypeDef* gpio__memory_map[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, NULL, NULL, GPIOH };

void gpio__port_clock_enable(gpio__port_e port)
{
  RCC->AHB1ENR |= (1UL << port);
}

gpio__gpio_s gpio__construct(gpio__port_e port, uint8_t pin)
{
  gpio__gpio_s gpio = {
    .port = port,
    .pin = pin
  };

  return gpio;
}

gpio__gpio_s gpio__configure_as_input(gpio__port_e port, uint8_t pin)
{
  gpio__gpio_s gpio = gpio__construct(port, pin);
  gpio__set_as_input(gpio);

  return gpio;
}

gpio__gpio_s gpio__configure_as_output(gpio__port_e port, uint8_t pin)
{
  gpio__gpio_s gpio = gpio__construct(port, pin);
  gpio__set_as_output(gpio);

  return gpio;
}

gpio__gpio_s gpio__configure_with_function(gpio__port_e port, uint8_t pin, gpio__alternate_function_e function)
{
  gpio__gpio_s gpio = gpio__construct(port, pin);
  gpio__set_as_alternate_function(gpio);
  gpio__set_function(gpio, function);
  return gpio;
}

void gpio__configure_speed(gpio__gpio_s gpio, gpio__speed_e speed)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;
  
  gpio_port->OSPEEDR &= ~(3UL << (gpio.pin * 2));
  gpio_port->OSPEEDR |= (speed << (gpio.pin * 2));
}

void gpio__set_function(gpio__gpio_s gpio, gpio__alternate_function_e function)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;

  if (gpio.pin < 8)
  {
    gpio_port->AFR[0] &= ~(0xF << (gpio.pin * 4));
    gpio_port->AFR[0] |= (function << (gpio.pin * 4));
  }
  else
  {
    gpio_port->AFR[1] &= ~(0xF << ((gpio.pin - 8) * 4));
    gpio_port->AFR[1] |= (function << ((gpio.pin - 8) * 4));
  }
}

void gpio__set_as_input(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;
  
  gpio_port->MODER &= ~(3UL << (gpio.pin * 2));
}

void gpio__set_as_output(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;
  
  gpio_port->MODER &= ~(3UL << (gpio.pin * 2));
  gpio_port->MODER |= (1UL << (gpio.pin * 2));
}

void gpio__set_as_alternate_function(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;
  
  gpio_port->MODER &= ~(3UL << (gpio.pin * 2));
  gpio_port->MODER |= (2UL << (gpio.pin * 2));
}

void gpio__enable_pull_up_resistor(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;
  
  gpio_port->PUPDR &= ~(3UL << (gpio.pin * 2));
  gpio_port->PUPDR |= (1UL << (gpio.pin * 2));
}

void gpio__enable_pull_down_resistor(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;
  
  gpio_port->PUPDR &= ~(3UL << (gpio.pin * 2));
  gpio_port->PUPDR |= (2UL << (gpio.pin * 2));
}

void gpio__set(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;
  
  gpio_port->BSRR = (1UL << gpio.pin);
}

void gpio__reset(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;
  
  gpio_port->BSRR = (1UL << (gpio.pin + 16));
}

bool gpio__get(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return 0;

  return (gpio_port->IDR & (1UL << gpio.pin));
}

void gpio__toggle(gpio__gpio_s gpio)
{
  GPIO_TypeDef* gpio_port = (GPIO_TypeDef*)gpio__memory_map[gpio.port];

  if (gpio_port == NULL || gpio.pin > 15)
    return;

  gpio_port->ODR ^= (1UL << gpio.pin);
}