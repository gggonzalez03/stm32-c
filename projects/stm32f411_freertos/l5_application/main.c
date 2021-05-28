#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "stm32f411xe.h"
#include "exti.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "bma400_spi.h"

void accelerometer_task(void *parameter);

SemaphoreHandle_t data_ready;

int main()
{
  data_ready = xSemaphoreCreateBinary();;

  xTaskCreate(accelerometer_task, "bma400 acc", 1024 / sizeof(void *), NULL, 1, NULL);

  vTaskStartScheduler();

  return 0;
}

void accelerometer_task(void *parameter)
{
  bma400_spi__axes_raw_s xyz_raw;
  bma400_spi__axes_mps2_s xyz_mps2;
  float x;

  bool bma_ok = bma400_spi__init();

  while (1)
  {
    if (!bma_ok)
    {
      printf("BMA400 initialization failed.\n");
      break;
    }

    if (xSemaphoreTake(data_ready, portMAX_DELAY))
    {
      xyz_mps2 = bma400_spi__get_acceleration_mps2();
      printf("x: %f, y: %f, z: %f\n", xyz_mps2.x, xyz_mps2.y, xyz_mps2.z);
    }
    else
    {
      printf("Data not ready.\n");
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