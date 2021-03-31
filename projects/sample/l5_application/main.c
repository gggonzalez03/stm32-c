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

static void config_usart_1();
static void config_usart_2();

void led_sender_task(void* parameter);
void led_receiver_task(void* parameter);

int main()
{

  message = xSemaphoreCreateBinary();

  xTaskCreate(led_sender_task, "led sender task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(led_receiver_task, "led receiver task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

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
  
  config_usart_1();
  config_usart_2();

  char a = 'A';
  char b = 'B';
  
  while (1)
  {
    if (xSemaphoreTake(message, portMAX_DELAY))
    {
      gpio__toggle(gpio);
      usart__polled_transmit(USART__1, a);
      usart__polled_transmit(USART__2, b);
    }
  }
}

static void config_usart_1()
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);
  (void)gpio__configure_with_function(GPIO__PORT_A, 9, GPIO__AF07);
  (void)gpio__configure_with_function(GPIO__PORT_A, 10, GPIO__AF07);
  usart__init(USART__1, clock__get_core_clock_frq(), 38400, false);
}

static void config_usart_2()
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);
  (void)gpio__configure_with_function(GPIO__PORT_A, 2, GPIO__AF07);
  (void)gpio__configure_with_function(GPIO__PORT_A, 3, GPIO__AF07);
  usart__init(USART__2, clock__get_core_clock_frq() / 2, 115200, false);
}