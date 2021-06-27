#include "silabs_ble_freertos.h"

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
#include "silabs_ble_gattdb.h"

volatile silabs_ble_freertos__buffer_t usart_rx_buffer;
static uint8_t advertising_set_handle = 0xFF;
static uint8_t connection_id = 0xFF;

QueueHandle_t usart_rx_queue;
xSemaphoreHandle usart_rx_buffer_mutex, ble_mutex;
xSemaphoreHandle usart_rx_buffer_not_empty_semphr;

/*************************************************************************
 * 
 *                      PRIVATE FUNCTION DEFINITIONS
 *
 *************************************************************************/

static void sl_bt_on_event(sl_bt_msg_t *evt);

static void silabs_ble_freertos__configure_usart(void)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_USART2, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);

  usart__init(USART__6, clock__get_core_clock_frq(), 115200, false);
  usart__enable_rx_interrrupt(USART__6);

  gpio__configure_with_function(GPIO__PORT_A, 11, GPIO__AF08); // tx pin
  gpio__configure_with_function(GPIO__PORT_A, 12, GPIO__AF08); // rx pin
}

static void silabs_ble_freertos__ncp_host_tx(uint32_t length, uint8_t *tx_bytes)
{
  for (uint32_t tx_byte_index = 0; tx_byte_index < length; tx_byte_index++)
  {
    usart__polled_transmit(USART__6, tx_bytes[tx_byte_index]);
  }
}

static int32_t silabs_ble_freertos__ncp_host_rx(uint32_t length, uint8_t *rx_buffer)
{
  int32_t result = -1;

  // Wait for the rx buffer to fill up. This 5ms duration is found experimentally and is a hack. This
  // is the shortest wait time that causes the least fails in reading the packet header. However, vTaskDelay
  // of 1ms also works. The downside is more iterations trying to fetch bytes that may not be there yet.
  // Generally 1ms delay seems to be faster. 0ms delay here for some reason, does not. I suspect it is
  // because the event/response header is not complete yet before sdk functions try to consume it using
  // this function
  vTaskDelay(5);

  if (usart_rx_buffer.length >= length)
  {
    xSemaphoreTake(usart_rx_buffer_mutex, portMAX_DELAY);
    memcpy((void *)rx_buffer, (void *)usart_rx_buffer.bytes, length);
    usart_rx_buffer.length -= length;
    memmove((void *)usart_rx_buffer.bytes, (void *)&usart_rx_buffer.bytes[length], usart_rx_buffer.length);
    result = length;
    xSemaphoreGive(usart_rx_buffer_mutex);
  }

  return result;
}

static int32_t silabs_ble_freertos__ncp_host_peek(void)
{
  return usart_rx_buffer.length;
}

static void silabs_ble_freertos__ble_main_task(void *parameter)
{
  sl_bt_msg_t evt;
  sl_status_t status;

  if (sl_bt_api_initialize_nonblock(silabs_ble_freertos__ncp_host_tx,
                                    silabs_ble_freertos__ncp_host_rx,
                                    silabs_ble_freertos__ncp_host_peek) != SL_STATUS_OK)
  {
    vTaskDelete(NULL);
  }

  sl_bt_system_reset(sl_bt_system_boot_mode_normal);

  while (true)
  {
    // Only attempt to pop an event when there is something in the tx buffer
    xSemaphoreTake(usart_rx_buffer_not_empty_semphr, portMAX_DELAY);

    // sl_bt_pop_event and sl_bt_on_event, like other sl_bt_* functions, call the ncp_host_rx function internally
    // the rx buffers and the bus itself need to be protected by a mutex
    // All other sl_bt_* functions need to be checked if they use ncp_host_rx. If so, take and give this mutex
    xSemaphoreTake(ble_mutex, portMAX_DELAY);
    status = sl_bt_pop_event(&evt);

    if (status == SL_STATUS_OK)
    {
      sl_bt_on_event(&evt);
    }
    xSemaphoreGive(ble_mutex);
  }
}

static void silabs_ble_freertos__rx_buffer_filler_task(void *parameter)
{
  uint8_t rx_byte;

  while (1)
  {
    if (xQueueReceive(usart_rx_queue, &rx_byte, portMAX_DELAY))
    {
      xSemaphoreTake(usart_rx_buffer_mutex, portMAX_DELAY);
      usart_rx_buffer.bytes[usart_rx_buffer.length] = rx_byte;
      usart_rx_buffer.length++;
      xSemaphoreGive(usart_rx_buffer_mutex);
      xSemaphoreGive(usart_rx_buffer_not_empty_semphr);
    }
  }
}

/*************************************************************************
 * 
 *                        PUBLIC FUNCTION DEFINITIONS
 *
 *************************************************************************/

static void silabs_ble_freertos__write_characteristic_task(void *parameter);
static void silabs_ble_freertos__write_characteristic_2_task(void *parameter);

void silabs_ble_freertos__initialize(unsigned long priority)
{
  silabs_ble_freertos__configure_usart();

  usart_rx_queue = xQueueCreate(1024, sizeof(uint8_t));
  usart_rx_buffer_mutex = xSemaphoreCreateMutex();
  ble_mutex = xSemaphoreCreateMutex();
  usart_rx_buffer_not_empty_semphr = xSemaphoreCreateBinary();

  xTaskCreate(silabs_ble_freertos__ble_main_task, "ble test task", 4096 / sizeof(void *), NULL, priority + 1, NULL);
  xTaskCreate(silabs_ble_freertos__write_characteristic_task, "ble write task", 2048 / sizeof(void *), NULL, priority, NULL);
  xTaskCreate(silabs_ble_freertos__write_characteristic_2_task, "ble write task", 2048 / sizeof(void *), NULL, priority, NULL);
  xTaskCreate(silabs_ble_freertos__rx_buffer_filler_task, "buffer filler task", 2048 / sizeof(void *), NULL, priority + 2, NULL);
}

bool silabs_ble_freertos__get_connection(uint8_t *id)
{
  if (connection_id == 0xFF)
  {
    return false;
  }

  *id = connection_id;
  return true;
}

bool silabs_ble_freertos__send_notification(uint8_t connection,
                                            uint16_t characteristic_handle,
                                            uint32_t length,
                                            const uint8_t *values)
{
  xSemaphoreTake(ble_mutex, portMAX_DELAY);
  sl_bt_gatt_server_send_notification(connection, characteristics[3].handle, length, values);
  xSemaphoreGive(ble_mutex);
}

/*************************************************************************
 * 
 *                            REQUIRED ISRS
 *
 *************************************************************************/
void USART6_IRQHandler(void)
{
  uint8_t rx_byte;

  if (USART6->SR & USART_SR_RXNE)
  {
    rx_byte = (uint8_t)USART6->DR;
    xQueueSendFromISR(usart_rx_queue, &rx_byte, NULL);
  }
}

/*************************************************************************
 * 
 *                                  TEST
 *
 *************************************************************************/
static void system_id_changed_callback(uint8array *value)
{
  uint8_t i;
  for (i = 0; i < value->len; i++)
  {
    printf("my_data[%d] = 0x%x \r\n", i, value->data[i]);
  }
  printf("--------------------------------\n");
}

static void system_id_write()
{
  static uint8_t values[10] = {0};
  uint32_t length = 10;

  sl_bt_gatt_server_send_notification(connection_id, characteristics[3].handle, length, values);

  for (size_t i = 0; i < 10; i++)
  {
    values[i]++;
  }
}

static void system_id_write_2()
{
  static uint8_t values[10] = {0};
  uint32_t length = 10;

  sl_bt_gatt_server_send_notification(connection_id, characteristics[2].handle, length, values);

  for (size_t i = 0; i < 10; i++)
  {
    values[i]++;
  }
}

static void silabs_ble_freertos__write_characteristic_task(void *parameter)
{
  while (1)
  {
    if (connection_id != 0xFF)
    {
      xSemaphoreTake(ble_mutex, portMAX_DELAY);
      system_id_write();
      xSemaphoreGive(ble_mutex);
    }
    vTaskDelay(100);
  }
}

static void silabs_ble_freertos__write_characteristic_2_task(void *parameter)
{
  while (1)
  {
    if (connection_id != 0xFF)
    {
      xSemaphoreTake(ble_mutex, portMAX_DELAY);
      system_id_write_2();
      xSemaphoreGive(ble_mutex);
    }
    vTaskDelay(300);
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
    printf("%d\n", evt->data.evt_connection_opened.connection);
    connection_id = evt->data.evt_connection_opened.connection;
    printf("Connection opened.\n");
    break;

  // -------------------------------
  // This event indicates that a connection was closed.
  case sl_bt_evt_connection_closed_id:
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
    if (characteristics[2].handle == evt->data.evt_gatt_server_attribute_value.attribute)
    {
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