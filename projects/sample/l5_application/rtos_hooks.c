#include "FreeRTOS.h"
#include "task.h"

// https://www.freertos.org/a00016.html

void vApplicationIdleHook( void )
{
  // sleep here
  ;
}

void vApplicationTickHook( void )
{
  ;
}

void vApplicationMallocFailedHook( void )
{
  ;
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
  ;
}
