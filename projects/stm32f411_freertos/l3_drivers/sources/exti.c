#include <stddef.h>

#include "exti.h"

#include "stm32f411xe.h"

static const volatile uint32_t *const trigger_edge_registers[2] = {&EXTI->FTSR, &EXTI->RTSR};

void exti__select_interrupt_line_source(uint8_t exti_number, uint8_t source)
{
  uint8_t control_register_index = exti_number / 4;

  if (control_register_index > 3) // there are only 4 control registers
  {
    return;
  }

  SYSCFG->EXTICR[control_register_index] &= ~(0xF << (exti_number * 4));
  SYSCFG->EXTICR[control_register_index] |= ((source & 0xF) << (exti_number * 4));
}

void exti__setup_external_interrupt(uint8_t exti_number, exti__trigger_edge_e edge)
{
  uint32_t *trigger_edge_register = (uint32_t *)trigger_edge_registers[edge];

  EXTI->IMR |= (1UL << exti_number);
  *trigger_edge_register |= (1UL << exti_number);
}

void exti__enable_external_interrupt(uint8_t exti_number)
{
  IRQn_Type irq_number = 0;

  if (exti_number >= 0 && exti_number <= 4)
  {
    irq_number = (IRQn_Type)(exti_number + 6);
  }
  else if (exti_number >= 5 && exti_number <= 9)
  {
    irq_number = EXTI9_5_IRQn;
  }
  else if (exti_number >= 10 && exti_number <= 15)
  {
    irq_number = EXTI15_10_IRQn;
  }

  if (irq_number == 0)
  {
    return;
  }

  NVIC_EnableIRQ(irq_number);
}

void exti__clear_external_interrupt(uint8_t exti_number)
{
  EXTI->PR |= (1UL << exti_number);
}