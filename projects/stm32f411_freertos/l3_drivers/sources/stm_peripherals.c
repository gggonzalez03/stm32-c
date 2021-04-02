
#include <stdlib.h>

#include "stm32f411xe.h"
#include "stm_peripherals.h"

static stm_peripheral__bus_e stm_peripheral__get_bus(stm_peripheral_e peripheral)
{
  if ((stm_peripheral__bus_e)peripheral >= STM_PERIPHERAL_APB2)
  {
    return STM_PERIPHERAL_APB2;
  }
  else if ((stm_peripheral__bus_e)peripheral >= STM_PERIPHERAL_APB1)
  {
    return STM_PERIPHERAL_APB1;
  }
  else if ((stm_peripheral__bus_e)peripheral >= STM_PERIPHERAL_AHB2)
  {
    return STM_PERIPHERAL_AHB2;
  }
  else if ((stm_peripheral__bus_e)peripheral >= STM_PERIPHERAL_AHB1)
  {
    return STM_PERIPHERAL_AHB1;
  }

  return -1;
}

bool stm_peripheral__power_on_peripheral(stm_peripheral_e peripheral, bool power_off_on_sleep)
{
  stm_peripheral__bus_e peripheral_bus = stm_peripheral__get_bus(peripheral);
  uint32_t power_on_bit = peripheral - peripheral_bus;
  uint32_t* bus_power_register;
  uint32_t* bus_low_power_register;

  switch (peripheral_bus)
  {
    case STM_PERIPHERAL_AHB1:
      bus_power_register = (uint32_t*)&RCC->AHB1ENR;
      bus_low_power_register = power_off_on_sleep ? (uint32_t*)&RCC->AHB1LPENR : NULL;
    break;

    case STM_PERIPHERAL_AHB2:
      bus_power_register = (uint32_t*)&RCC->AHB2ENR;
      bus_low_power_register = power_off_on_sleep ? (uint32_t*)&RCC->AHB2LPENR : NULL;
    break;

    case STM_PERIPHERAL_APB1:
      bus_power_register = (uint32_t*)&RCC->APB1ENR;
      bus_low_power_register = power_off_on_sleep ? (uint32_t*)&RCC->APB1LPENR : NULL;
    break;

    case STM_PERIPHERAL_APB2:
      bus_power_register = (uint32_t*)&RCC->APB2ENR;
      bus_low_power_register = power_off_on_sleep ? (uint32_t*)&RCC->APB2LPENR : NULL;
    break;

    default:
      bus_power_register = NULL;
      bus_low_power_register = NULL;
    break;
  }

  if (bus_power_register != NULL)
  {
    *bus_power_register |= (1UL << power_on_bit);
  }

  if (bus_low_power_register != NULL)
  {
    // Clear the bit to disable peripheral clock (See Chapter 5.3.2 of the Reference Manual)
    *bus_low_power_register &= ~(1UL << power_on_bit);
  }

  return true;
}

bool stm_peripheral__is_powered_on(stm_peripheral_e peripheral)
{
  stm_peripheral__bus_e peripheral_bus = stm_peripheral__get_bus(peripheral);
  uint32_t power_on_bit = peripheral - peripheral_bus;
  uint32_t* bus_power_register;

  switch (peripheral_bus)
  {
    case STM_PERIPHERAL_AHB1:
      bus_power_register = (uint32_t*)&RCC->AHB1ENR;
    break;

    case STM_PERIPHERAL_AHB2:
      bus_power_register = (uint32_t*)&RCC->AHB2ENR;
    break;

    case STM_PERIPHERAL_APB1:
      bus_power_register = (uint32_t*)&RCC->APB1ENR;
    break;

    case STM_PERIPHERAL_APB2:
      bus_power_register = (uint32_t*)&RCC->APB2ENR;
    break;

    default:
      bus_power_register = NULL;
    break;
  }

  return (*bus_power_register & (1U << power_on_bit));
}

bool stm_peripheral__is_powered_on_in_sleep_mode(stm_peripheral_e peripheral)
{
  stm_peripheral__bus_e peripheral_bus = stm_peripheral__get_bus(peripheral);
  uint32_t power_on_bit = peripheral - peripheral_bus;
  uint32_t* bus_low_power_register;

  switch (peripheral_bus)
  {
    case STM_PERIPHERAL_AHB1:
      bus_low_power_register = (uint32_t*)&RCC->AHB1LPENR;
    break;

    case STM_PERIPHERAL_AHB2:
      bus_low_power_register = (uint32_t*)&RCC->AHB2LPENR;
    break;

    case STM_PERIPHERAL_APB1:
      bus_low_power_register = (uint32_t*)&RCC->APB1LPENR;
    break;

    case STM_PERIPHERAL_APB2:
      bus_low_power_register = (uint32_t*)&RCC->APB2LPENR;
    break;

    default:
      bus_low_power_register = NULL;
    break;
  }

  return (*bus_low_power_register & (1U << power_on_bit));
}