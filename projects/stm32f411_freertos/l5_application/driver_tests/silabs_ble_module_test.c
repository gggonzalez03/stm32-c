/**
 * First, the BLE stack firmware has to be loaded into the EFR32MG12 (EFR32BG22 could be used as well, but
 * need testing and verification). The firmware is so that the device can act as network coprocessor to an
 * external microcontroller. In our case, that is STM32F4.
 * 
 * To load the firmware:
 * 1. Download, install, and open Simplicity Studio 5 from siliconlabs website
 * 2. If you have an evaluation board, such as Thunderboard Sense 2, plug that in.
 *    If not, you will need a USB to UART converter, or JTAG connector and figure out how to upload firmware
 * 3. Create a new project
 * 4. Choose "Bluetooth - NCP" example project
 * 5. Change the UART pins to use
 * 6. Upload firware to the device
 * 
 * You can verify that the module is working using the "Bluetooth NCP Commander" tool on the Simplicity Studio.
 * Use a USB to UART converter for this to connect to the BLE device
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
#include "semphr.h"

#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"
#include "gatt_database.h"

typedef struct
{
  uint16_t length;
  uint8_t bytes[1024];
} ble_buffer_t;

volatile ble_buffer_t usart_rx_buffer;
QueueHandle_t usart_rx_queue;
xSemaphoreHandle rx_buffer_mutex;
xSemaphoreHandle event_pending_semphr;

static uint8_t advertising_set_handle = 0xff;
bool connection_open = false;
uint8_t connection = 0xFF;

static void ncp_host_tx(uint32_t length, uint8_t *tx_bytes);
static int32_t ncp_host_rx(uint32_t length, uint8_t *rx_buffer);
static int32_t ncp_host_peek(void);
static void sl_bt_on_event(sl_bt_msg_t *evt);

static void system_id_changed_callback(uint8array *value);
static void system_id_write();

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

    xSemaphoreTake(event_pending_semphr, portMAX_DELAY);

    status = sl_bt_pop_event(&evt);
    if (status != SL_STATUS_OK)
    {
      continue;
    }

    sl_bt_on_event(&evt);
  }
}

void silabs_ble_module_test__rx_buffer_filler_task(void *parameter)
{
  uint8_t rx_byte;

  usart_rx_queue = xQueueCreate(1024, sizeof(uint8_t));
  rx_buffer_mutex = xSemaphoreCreateMutex();
  event_pending_semphr = xSemaphoreCreateBinary();

  while (1)
  {
    if (xQueueReceive(usart_rx_queue, &rx_byte, portMAX_DELAY))
    {
      xSemaphoreTake(rx_buffer_mutex, portMAX_DELAY);
      usart_rx_buffer.bytes[usart_rx_buffer.length] = rx_byte;
      usart_rx_buffer.length++;
      xSemaphoreGive(rx_buffer_mutex);
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

    // Pad and reverse unique ID to get System ID.
    characteristics[SYSTEM_ID].value[0] = address.addr[5];
    characteristics[SYSTEM_ID].value[1] = address.addr[4];
    characteristics[SYSTEM_ID].value[2] = address.addr[3];
    characteristics[SYSTEM_ID].value[3] = 0xFF;
    characteristics[SYSTEM_ID].value[4] = 0xFE;
    characteristics[SYSTEM_ID].value[5] = address.addr[2];
    characteristics[SYSTEM_ID].value[6] = address.addr[1];
    characteristics[SYSTEM_ID].value[7] = address.addr[0];

    gatt_database__initialize();

    ble_status = sl_bt_advertiser_create_set(&advertising_set_handle);
    // Set advertising interval to 100ms.
    ble_status = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
    // Start general advertising and enable connections.
    ble_status = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);

    if (ble_status == SL_STATUS_OK)
    {
      printf("Started advertising.\n");
    }

    break;

  // -------------------------------
  // This event indicates that a new connection was opened.
  case sl_bt_evt_connection_opened_id:
    connection_open = true;
    printf("%d\n", evt->data.evt_connection_opened.connection);
    connection = evt->data.evt_connection_opened.connection;
    printf("Connection opened.\n");
    break;

  // -------------------------------
  // This event indicates that a connection was closed.
  case sl_bt_evt_connection_closed_id:
    connection_open = false;
    printf("Connection closed.\n");
    // Restart advertising after client has disconnected.
    ble_status = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);

    if (ble_status == SL_STATUS_OK)
    {
      printf("Started advertising after disconnect.\n");
    }
    break;
  case sl_bt_evt_gatt_server_attribute_value_id:
    // Check if the event is because of the my_data changed by the remote GATT client
    if (characteristics[3].handle == evt->data.evt_gatt_server_attribute_value.attribute)
    {
      // Call my handler
      system_id_changed_callback(&(evt->data.evt_gatt_server_attribute_value.value));
    }
    break;
  case sl_bt_evt_gatt_server_characteristic_status_id:
    // printf("Characteristic Event Code: %lx\n", SL_BT_MSG_ID(evt->header));
    break;

  default:
    // printf("%lx\n", SL_BT_MSG_ID(evt->header));
    break;
  }
}

static void system_id_changed_callback(uint8array *value)
{
  uint8_t i;
  for (i = 0; i < value->len; i++)
  {
    printf("my_data[%d] = 0x%x \r\n", i, value->data[i]);
  }
  printf("Received data.\n");
}

static void system_id_write()
{
  static uint8_t values[10] = { 0 };
  uint32_t length = 10;

  sl_bt_gatt_server_send_notification(connection, characteristics[3].handle, length, values);
  
  for (size_t i = 0; i < 10; i++)
  {
    values[i]++;
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

  // Wait for the rx buffer to fill up. This 4ms duration is found experimentally and is a hack. This
  // is the shortest wait time that causes the least fails in reading the packet header
  vTaskDelay(5);

  if (usart_rx_buffer.length >= length)
  {
    xSemaphoreTake(rx_buffer_mutex, portMAX_DELAY);
    memcpy((void *)rx_buffer, (void *)usart_rx_buffer.bytes, length);
    usart_rx_buffer.length -= length;
    memmove((void *)usart_rx_buffer.bytes, (void *)&usart_rx_buffer.bytes[length], usart_rx_buffer.length);
    result = length;
    xSemaphoreGive(rx_buffer_mutex);
  }

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

  // xSemaphoreGive(event_pending_semphr);
  xSemaphoreGiveFromISR(event_pending_semphr, NULL);
}