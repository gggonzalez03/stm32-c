#include <stdint.h>

#include "stm32f411xe.h"

#include "stm_peripherals.h"
#include "FreeRTOS.h"
#include "clock.h"
#include "flash.h"
#include "usart.h"
#include "gpio.h"

static void startup__init_usart1(void)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);
  (void)gpio__configure_with_function(GPIO__PORT_A, 9, GPIO__AF07);
  (void)gpio__configure_with_function(GPIO__PORT_A, 10, GPIO__AF07);

  usart__init(USART__1, clock__get_core_clock_frq(), 115200, false);
}

void startup__init_data_sram(void)
{
  extern uint32_t __text_end__;
  extern uint32_t __data_end__;
  extern uint32_t __data_start__;

  uint8_t *src_flash = (uint8_t*)&__text_end__;
  uint8_t *dest_sram = (uint8_t*)&__data_start__;

  while (dest_sram < (uint8_t*)&__data_end__) {
    *dest_sram++ = *src_flash++;
  }
}

void startup__init_bss_sram(void)
{
  extern uint32_t __bss_start__;
  extern uint32_t __bss_end__;

  uint8_t *bss_sram_start = (uint8_t*)&__bss_start__;
  while (bss_sram_start < (uint8_t*)&__bss_end__) {
    *bss_sram_start++ = 0U;
  }
}

// See Chapter 4.6.6 of the Programmer's Manual
void startup__init_fpu(void)
{
  SCB->CPACR |= (0xF << 20);
}

void startup__init_interrupts(void)
{
  /**
   * Set all peripherals' priorities to lower than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
   * from the FreeRTOSConfig.h. Any interrupt that has a higher priority than that cannot use 
   * FreeRTOS API, not even the FromISRs.
   * 
   * Note that lower priority means numerically higher number.
   * TODO: Test if the +1 could be removed here.
   **/
  const uint32_t peripheral_interrupt_priority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1;

  const int32_t first_peripheral = (int32_t)WWDG_IRQn;
  const int32_t last_peripheral = (int32_t)SPI5_IRQn;

  for (int32_t peripheral = first_peripheral; peripheral <= last_peripheral; peripheral++)
  {
    NVIC_SetPriority(peripheral, peripheral_interrupt_priority);
  }
}

void startup__init_system(void)
{
  /*****************************************************************
   * These two lines are required
   ****************************************************************/
  startup__init_data_sram();
  startup__init_bss_sram();

  /*****************************************************************
   * These two lines can be removed, but clock__get_core_clock_frq()
   * from the clock.c file should be modified to reflect
   * the default clock frequency. Note that if one of them is
   * removed, the other should removed as well
   ****************************************************************/
  flash__config_3v_frq_64_90MHz();
  clock__init_system_clock();

  /*****************************************************************
   * These are really optional, one could also add
   * some more function calls here from other modules
   ****************************************************************/
  startup__init_fpu();
  startup__init_interrupts();
  startup__init_usart1();
}