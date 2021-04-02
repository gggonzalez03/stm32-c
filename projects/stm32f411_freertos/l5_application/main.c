#include <stdint.h>
#include <stdio.h>

#include "stm32f411xe.h"

#include "stm_peripherals.h"
#include "clock.h"
#include "gpio.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static SemaphoreHandle_t message;

void led_sender_task(void* parameter);
void led_receiver_task(void* parameter);

int main()
{
  message = xSemaphoreCreateBinary();

  xTaskCreate(led_sender_task, "sender", 512 / sizeof(void *), NULL, 1, NULL);
  xTaskCreate(led_receiver_task, "receiver", 1024 / sizeof(void *), NULL, 1, NULL);

  vTaskStartScheduler();

  return 0;
}

void led_sender_task(void* parameter)
{
  while (1)
  {
    xSemaphoreGive(message);
    vTaskDelay(300);
  }
}

void led_receiver_task(void* parameter)
{
  static uint8_t led_0 = 15;
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOC, false);
  gpio__gpio_s gpio = gpio__configure_as_output(GPIO__PORT_C, led_0);
  
  while (1)
  {
    if (xSemaphoreTake(message, portMAX_DELAY))
    {
      printf("Toggling LED\n");
      gpio__toggle(gpio);
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