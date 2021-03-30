#include <stdint.h>
#include <stdio.h>

#include "stm32f411xe.h"

#include "gpio.h"
#include "clock.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static SemaphoreHandle_t message;

static void crude_delay_ms(uint32_t ms)
{
  uint32_t count_up_to = TIM2->CNT + (ms * 1000);
  while (TIM2->CNT < count_up_to);
}

static void timer__enable_counter()
{
  uint32_t timer_2_enable = (1UL << 0);
  uint32_t start_counter = (1UL << 0);
  uint32_t generate_update_event = (1UL << 0);

  RCC->APB1ENR |= timer_2_enable;
  /**
   * Clocks on timers on APB1 bus are multiplied by 2 if the APB1 bus
   * prescaler is greater than 1. So basically the minimum frequency of
   * timer clocks is the AHB bus frequency
   * See Chapter 6.2, Figure 12.
   **/ 
  TIM2->PSC = (clock__get_core_clock_frq() / (1000UL * 1000UL) - 1UL);
  TIM2->CR1 |= start_counter;
  TIM2->EGR |= generate_update_event; // Required for the prescaler value to take effect
}

void uart__configure_uart_1()
{
  uint32_t uart_1_clock_enable = (1UL << 4);
  RCC->APB2ENR |= uart_1_clock_enable;

  gpio__gpio_s tx = gpio__configure_with_function(GPIO__PORT_A, 9, GPIO__AF07);
  gpio__gpio_s rx = gpio__configure_with_function(GPIO__PORT_A, 10, GPIO__AF07);

  /**
   * 84000000 / (16 * 115200) = 45.5729166667
   * 45 => 0x02D
   * .5729166667*16 => 9.17 => 9 => 0x9
   * BRR[15:0] = 0x02D9
   **/
  USART1->BRR = 0x02D9;

  uint32_t receiver_enable = (1UL << 2);
  uint32_t transmitter_enable = (1UL << 3);
  USART1->CR1 |= (transmitter_enable);
  USART1->CR1 |= (receiver_enable);

  uint32_t uart_1_enable = (1UL << 13);
  USART1->CR1 |= uart_1_enable;
}

void uart__transfer_byte(char byte)
{
  USART1->DR = byte & 0xFF;
  while (!(USART1->SR & (1UL << 6)));
}

void uart__receive_byte(char* byte)
{
  while (!(USART1->SR & (1UL << 5)));
  *byte = (char)USART1->DR;
}

void mco_1_enable()
{
  gpio__port_clock_enable(GPIO__PORT_A);
  gpio__gpio_s mco_1 = gpio__configure_with_function(GPIO__PORT_A, 8, GPIO__AF00);
  gpio__configure_speed(mco_1, GPIO__50MHz);
  uint32_t mco_1_prescaler = (0b111 << 24);
  uint32_t mco_1_select_pll = (3UL << 21);
  RCC->CFGR |= (mco_1_prescaler | mco_1_select_pll);
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
  uint8_t* pin = (uint8_t*)parameter;
  gpio__gpio_s gpio = gpio__configure_as_output(GPIO__PORT_C, *pin);
  
  while (1)
  {
    if (xSemaphoreTake(message, portMAX_DELAY))
    {
      gpio__toggle(gpio);
    }
  }
}

int main()
{
  static uint8_t led_0 = 15;
  gpio__port_clock_enable(GPIO__PORT_C);
  gpio__gpio_s gpio_0 = gpio__configure_as_output(GPIO__PORT_C, led_0);

  message = xSemaphoreCreateBinary();

  xTaskCreate(led_sender_task, "led sender task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(led_receiver_task, "led receiver task", configMINIMAL_STACK_SIZE, &led_0, 1, NULL);

  vTaskStartScheduler();

  return 0;
}