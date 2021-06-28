#include <stdint.h>
#include <stdbool.h>

#include "stm32f411xe.h"
#include "exti.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "bma400_spi.h"

#include "silabs_ble_freertos.h"
#include "silabs_ble_gattdb.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

void accelerometer_task(void *parameter);
void silabs_ble_freertos__write_characteristic_task(void *parameter);

SemaphoreHandle_t data_ready;
QueueHandle_t z_queue;

int main()
{
  data_ready = xSemaphoreCreateBinary();
  z_queue = xQueueCreate(10, sizeof(uint8_t));

  silabs_ble_freertos__initialize(4);
  xTaskCreate(silabs_ble_freertos__write_characteristic_task, "write task", 1024 / sizeof(void*), NULL, 1, NULL);
  xTaskCreate(accelerometer_task, "accel task", 4096 / sizeof(void*), NULL, 1, NULL);

  vTaskStartScheduler();

  return 0;
}

void accelerometer_task(void *parameter)
{
  float z;
  uint8_t z_int;

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
      z_int = (uint8_t)z;

      PRINTF("z: %f\n", z);

      xQueueSend(z_queue, (void *)&z_int, portMAX_DELAY);
    }
    else
    {
      PRINTF("Data not ready.\n");
    }

    vTaskDelay(100);
  }
}

void silabs_ble_freertos__write_characteristic_task(void *parameter)
{
  uint8_t connection_id;
  uint8_t z;
  uint32_t length = 1;

  while (1)
  {
    if (silabs_ble_freertos__get_connection(&connection_id))
    {
      xQueueReceive(z_queue, &z, portMAX_DELAY);
      silabs_ble_freertos__send_notification(connection_id, characteristics[2].handle, length, &z);
    }
    else
    {
      vTaskDelay(50);
    }
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