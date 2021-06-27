#include <stdint.h>
#include <stdbool.h>

#include "stm32f411xe.h"
#include "exti.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "bma400_spi.h"

#include "silabs_ble_freertos.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

void accelerometer_task(void *parameter);

SemaphoreHandle_t data_ready;

int main()
{
  data_ready = xSemaphoreCreateBinary();;

  silabs_ble_freertos__initialize(1);

  vTaskStartScheduler();

  return 0;
}

void accelerometer_task(void *parameter)
{
  float z;

  bool bma_ok = bma400_spi__init();

  while (1)
  {
    if (!bma_ok)
    {
      PRINTF("BMA400 initialization failed.\n");
      break;
    }

    if (xSemaphoreTake(data_ready, portMAX_DELAY))
    {
      z = bma400_spi__get_z_mps2();
      PRINTF("z: %f\n", z);
    }
    else
    {
      PRINTF("Data not ready.\n");
    }

    vTaskDelay(100);
  }
}

void USART1_IRQHandler(void)
{
  if (USART1->SR & USART_SR_RXNE)
  {
    USART1->DR = USART1->DR;
  }
}

void EXTI3_IRQHandler(void)
{
  xSemaphoreGiveFromISR(data_ready, NULL);
  exti__clear_external_interrupt(3);
}