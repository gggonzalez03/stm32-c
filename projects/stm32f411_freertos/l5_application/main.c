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

static void spi__configure(void)
{
  gpio__configure_with_function(GPIO__PORT_A, 5, GPIO__AF05);
  gpio__configure_with_function(GPIO__PORT_A, 6, GPIO__AF05);
  gpio__configure_with_function(GPIO__PORT_A, 7, GPIO__AF05);

  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_SPI1, false);
  
  // Steps followed are outlined in the Reference Manual
  // Chapter 20.3.3
  uint32_t spi_baud_divider_mask = (7UL << 3);
  uint32_t spi_baud_divider_32 = (4UL << 3);
  uint32_t spi_8_bit_data_frame = (1UL << 11);
  uint32_t spi_master_mode = (1UL << 6);
  uint32_t spi_enable = (1UL << 2);
  uint32_t spi_software_slave_mgnt = (1UL << 9);
  // This is set in conjunction with the software slave management bit. See here why SSOE has to be set:
  // https://electronics.stackexchange.com/questions/442794/why-ss-pin-must-be-set-to-output-for-spi-master
  uint32_t spi_slave_select_output_enable = (1UL << 2);

  SPI1->CR1 &= ~(spi_baud_divider_mask);
  SPI1->CR1 |= spi_baud_divider_32;
  SPI1->CR1 &= ~(spi_8_bit_data_frame);
  SPI1->CR2 |= spi_slave_select_output_enable;
  SPI1->CR1 |= spi_master_mode | spi_enable | spi_software_slave_mgnt;
}

static uint8_t spi_exchange_byte(uint8_t byte)
{
  uint32_t spi_busy = (1UL << 7);
  uint32_t spi_transmit_buffer_empty = (1UL << 1);

  while (!(SPI1->SR & spi_transmit_buffer_empty));

  SPI1->DR = byte & 0xFF;

  while (!(SPI1->SR & spi_busy));
  while (SPI1->SR & spi_busy);

  return (SPI1->DR & 0xFF);
}

static void spi_transmit_bytes(uint8_t* bytes, uint32_t count)
{
  uint32_t spi_busy = (1UL << 7);
  uint32_t spi_transmit_buffer_empty = (1UL << 1);

  for (uint32_t current_byte = 0; current_byte < count; current_byte++)
  {
    while (!(SPI1->SR & spi_transmit_buffer_empty));
    SPI1->DR = bytes[current_byte] & 0xFF;
  }

  while (!(SPI1->SR & spi_busy));
  while (SPI1->SR & spi_busy);
}

static SemaphoreHandle_t message;

void led_sender_task(void* parameter);
void led_receiver_task(void* parameter);

int main()
{
  message = xSemaphoreCreateBinary();

  spi__configure();

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
  uint8_t received_byte;
  uint8_t bytes_to_transfer[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  
  while (1)
  {
    if (xSemaphoreTake(message, portMAX_DELAY))
    {
      gpio__reset(gpio);
      received_byte = spi_exchange_byte(9);
      spi_transmit_bytes(bytes_to_transfer, 10);
      gpio__set(gpio);

      printf("Byte received: %d\n", received_byte);
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