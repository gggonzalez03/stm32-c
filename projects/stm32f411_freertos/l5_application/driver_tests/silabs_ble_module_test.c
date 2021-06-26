

/**
 * First, the BLE stack firmware has to be loaded into the EFR32MG12 (EFR32BG22 could be used as well, but
 * need testing and verification). The firmware is so that the device can act as network coprocessor to an
 * external microcontroller. In our case, that is STM32F4.
 * 
 * To load the firmware:
 * 1. Download, install, and open Simplicity Studio 5 from siliconlabs website
 * 2. If you have an evaluation board, such as Thunderboard Sense 2, plug that in.
 *    If not, you will need a USB to UART converter, or JTAG connector and figure out how to upload firware
 * 3. Create a new project
 * 4. Choose "Bluetooth - NCP Empty" example project
 * 5. Change the UART pins to use
 * 6. Upload firware to the device
 * 
 * You can verify that the module is working using the "Bluetooth NCP Commander" tool on the Simplicity Studio.
 * You can use a USB to UART converter for this to connect to the BLE device
 * 
 * After this, the device can be used as a network coprocessor using siliconlab's bluetooth APIs
 **/

#include <stdint.h>
#include <stdio.h>

#include "silabs_ble_module_test.h"

#include "stm32f411xe.h"
#include "stm_peripherals.h"
#include "usart.h"
#include "gpio.h"
#include "clock.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"

typedef struct
{
  uint16_t length;
  uint8_t bytes[1024];
} ble_buffer_t;

volatile ble_buffer_t usart_rx_buffer;
QueueHandle_t usart_rx_queue;

static void ncp_host_tx(uint32_t length, uint8_t *tx_bytes);
static int32_t ncp_host_rx(uint32_t length, uint8_t *rx_buffer);
static int32_t ncp_host_peek(void);
static void sl_bt_on_event(sl_bt_msg_t *evt);

static void silabs_ble_module_test__configure(void)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_USART2, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);

  usart__init(USART__6, clock__get_core_clock_frq(), 115200, false);
  usart__enable_rx_interrrupt(USART__6);

  gpio__configure_with_function(GPIO__PORT_A, 11, GPIO__AF08); // tx pin
  gpio__configure_with_function(GPIO__PORT_A, 12, GPIO__AF08); // rx pin
}

void silabs_ble_module_test__task(void *parameter)
{
  sl_status_t ble_status;

  ble_status = sl_bt_api_initialize_nonblock(ncp_host_tx, ncp_host_rx, ncp_host_peek);
  silabs_ble_module_test__configure();

  if (ble_status == SL_STATUS_OK)
  {
    sl_bt_system_reset(sl_bt_system_boot_mode_normal);
  }
  else
  {
    vTaskDelete(NULL);
  }

  sl_bt_msg_t evt;
  sl_status_t status;

  while (1)
  {
    status = sl_bt_pop_event(&evt);
    if (status != SL_STATUS_OK)
    {
      // printf("Hello: %ld\n", status);
      // vTaskDelay(2000);
      continue;
    }

    sl_bt_on_event(&evt);
  }
}

void silabs_ble_module_test__rx_buffer_filler_task(void *parameter)
{
  uint8_t rx_byte;

  usart_rx_queue = xQueueCreate(1024, sizeof(uint8_t));

  while (1)
  {
    if (xQueueReceive(usart_rx_queue, &rx_byte, portMAX_DELAY))
    {
      /**
       * TODO:
       * Add a mutex here
       **/
      usart_rx_buffer.bytes[usart_rx_buffer.length] = rx_byte;
      usart_rx_buffer.length++;
    }
  }
}

static void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t ble_status;
  bd_addr address;
  uint8_t address_type;

  switch (SL_BT_MSG_ID(evt->header))
  {
  case sl_bt_evt_system_boot_id:
    printf("Bluetooth stack booted: v%d.%d.%d-b%d\n",
           evt->data.evt_system_boot.major,
           evt->data.evt_system_boot.minor,
           evt->data.evt_system_boot.patch,
           evt->data.evt_system_boot.build);
    ble_status = sl_bt_system_get_identity_address(&address, &address_type);
    if (ble_status == SL_STATUS_OK)
    {
      printf("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
             address_type ? "static random" : "public device",
             address.addr[5],
             address.addr[4],
             address.addr[3],
             address.addr[2],
             address.addr[1],
             address.addr[0]);
    }
    break;

  default:
    printf("%lx\n", SL_BT_MSG_ID(evt->header));
    break;
  }
}

static void ncp_host_tx(uint32_t length, uint8_t *tx_bytes)
{
  for (uint32_t tx_byte_index = 0; tx_byte_index < length; tx_byte_index++)
  {
    usart__polled_transmit(USART__6, tx_bytes[tx_byte_index]);
  }
}

static int32_t ncp_host_rx(uint32_t length, uint8_t *rx_buffer)
{
  int32_t result = -1;
  if (usart_rx_buffer.length >= length)
  {
    /**
     * TODO:
     * Add a mutex here
     **/
    memcpy((void *)rx_buffer, (void *)usart_rx_buffer.bytes, length);
    usart_rx_buffer.length -= length;
    memmove((void *)usart_rx_buffer.bytes, (void *)&usart_rx_buffer.bytes[length], usart_rx_buffer.length);
    result = length;
  }

  printf("Received some stuff\n");

  return result;
}

static int32_t ncp_host_peek(void)
{
  return usart_rx_buffer.length;
}

void USART6_IRQHandler(void)
{
  uint8_t rx_byte;

  if (USART6->SR & USART_SR_RXNE)
  {
    rx_byte = (uint8_t)USART6->DR;
    xQueueSendFromISR(usart_rx_queue, &rx_byte, NULL);
  }
}