#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

// https://www.freertos.org/a00016.html

void vApplicationIdleHook( void )
{
  /**
   * Peripherals that are allowed to be off will automatically turn off
   * This is determined by two registers in STM32F411: AHBxLPENRR and APBxLPENR
   * where x is the bus number.
   * See Chapter 5.3.2 of the Reference Manual
   **/
  __asm__("WFI");
}

void vApplicationTickHook( void )
{
  ;
}

void vApplicationMallocFailedHook( void )
{
  printf("Malloc failed\n");
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
  printf("Stack Overflow: %s\n", pcTaskName);
  while (1);
}
