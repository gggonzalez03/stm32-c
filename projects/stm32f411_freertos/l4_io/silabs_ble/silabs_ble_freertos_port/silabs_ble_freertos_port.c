#include <stdio.h>

#include "silabs_ble_freertos_port.h"

#include "stm32f411xe.h"
#include "stm_peripherals.h"
#include "usart.h"
#include "gpio.h"
#include "clock.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"
#include "silabs_ble_gattdb.h"

extern QueueHandle_t usart_rx_queue;

void silabs_ble_freertos__configure_usart(void)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_USART2, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);

  usart__init(USART__2, clock__get_apb1_clock_frq(), 115200, false);
  usart__enable_rx_interrrupt(USART__2);
  usart__enable_hardware_flow_control(USART__2);

  gpio__configure_with_function(GPIO__PORT_A, 0, GPIO__AF07); // cts pin
  gpio__configure_with_function(GPIO__PORT_A, 1, GPIO__AF07); // rts pin
  gpio__configure_with_function(GPIO__PORT_A, 2, GPIO__AF07); // tx pin
  gpio__configure_with_function(GPIO__PORT_A, 3, GPIO__AF07); // rx pin
}

void silabs_ble_freertos__uart_transmit_one_byte(uint8_t byte)
{
  usart__polled_transmit(USART__2, byte);
}

void USART2_IRQHandler(void)
{
  uint8_t rx_byte;

  if (USART2->SR & USART_SR_RXNE)
  {
    rx_byte = (uint8_t)USART2->DR;
    xQueueSendFromISR(usart_rx_queue, &rx_byte, NULL);
  }
}