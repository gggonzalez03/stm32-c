
#include <stdlib.h>

#include "stm32f411xe.h"
#include "stm_peripherals.h"

static uint32_t stm_peripheral__get_bus(stm_peripheral_e peripheral)
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

bool stm_peripheral__power_on_peripheral(stm_peripheral_e peripheral, bool low_power_mode)
{
  uint32_t peripheral_bus = stm_peripheral__get_bus(peripheral);
  uint32_t power_on_bit = peripheral - peripheral_bus;
  uint32_t* bus_power_register;

  switch (peripheral_bus)
  {
    case STM_PERIPHERAL_AHB1:
      bus_power_register = (low_power_mode == true ? (uint32_t*)&RCC->AHB1LPENR : (uint32_t*)&RCC->AHB1ENR);
    break;

    case STM_PERIPHERAL_AHB2:
      bus_power_register = (low_power_mode == true ? (uint32_t*)&RCC->AHB2LPENR : (uint32_t*)&RCC->AHB2ENR);
    break;

    case STM_PERIPHERAL_APB1:
      bus_power_register = (low_power_mode == true ? (uint32_t*)&RCC->APB1LPENR : (uint32_t*)&RCC->APB1ENR);
    break;

    case STM_PERIPHERAL_APB2:
      bus_power_register = (low_power_mode == true ? (uint32_t*)&RCC->APB2LPENR : (uint32_t*)&RCC->APB2ENR);
    break;

    default:
      bus_power_register = NULL;
    break;
  }

  if (bus_power_register != NULL)
  {
    *bus_power_register |= (1U << power_on_bit);
    return true;
  }

  return false;
}

bool stm_peripheral__is_powered_on(stm_peripheral_e peripheral, bool low_power_mode)
{
  uint32_t peripheral_bus = stm_peripheral__get_bus(peripheral);
  uint32_t power_on_bit = peripheral - peripheral_bus;
  uint32_t* bus_power_register;

  switch (peripheral_bus)
  {
    case STM_PERIPHERAL_AHB1:
      bus_power_register = (low_power_mode == true ? (uint32_t*)&RCC->AHB1LPENR : (uint32_t*)&RCC->AHB1ENR);
    break;

    case STM_PERIPHERAL_AHB2:
      bus_power_register = (low_power_mode == true ? (uint32_t*)&RCC->AHB2LPENR : (uint32_t*)&RCC->AHB2ENR);
    break;

    case STM_PERIPHERAL_APB1:
      bus_power_register = (low_power_mode == true ? (uint32_t*)&RCC->APB1LPENR : (uint32_t*)&RCC->APB1ENR);
    break;

    case STM_PERIPHERAL_APB2:
      bus_power_register = (low_power_mode == true ? (uint32_t*)&RCC->APB2LPENR : (uint32_t*)&RCC->APB2ENR);
    break;

    default:
      bus_power_register = NULL;
    break;
  }

  return (*bus_power_register & (1U << power_on_bit));
}