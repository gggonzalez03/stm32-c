#include "silabs_ble_freertos.h"
#include "silabs_ble_freertos_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"

volatile silabs_ble_freertos__buffer_t usart_rx_buffer;
static silabs_ble_freertos__on_event_callback_f on_event = NULL;

QueueHandle_t usart_rx_queue;
xSemaphoreHandle usart_rx_buffer_mutex, ble_mutex;
xSemaphoreHandle usart_rx_buffer_not_empty_semphr;

/*************************************************************************
 * 
 *                      PRIVATE FUNCTION DEFINITIONS
 *
 *************************************************************************/
static void silabs_ble_freertos__ncp_host_tx(uint32_t length, uint8_t *tx_bytes)
{
  for (uint32_t tx_byte_index = 0; tx_byte_index < length; tx_byte_index++)
  {
    silabs_ble_freertos__uart_transmit_one_byte(tx_bytes[tx_byte_index]);
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
  vTaskDelay(1);

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

    if (on_event == NULL)
    {
      continue;
    }

    // sl_bt_pop_event and sl_bt_on_event, like other sl_bt_* functions, call the ncp_host_rx function internally
    // the rx buffers and the bus itself need to be protected by a mutex
    // All other sl_bt_* functions need to be checked if they use ncp_host_rx. If so, take and give this mutex
    xSemaphoreTake(ble_mutex, portMAX_DELAY);
    status = sl_bt_pop_event(&evt);

    if (status == SL_STATUS_OK)
    {
      (*on_event)(&evt);
    }
    xSemaphoreGive(ble_mutex);
  }
}

static void silabs_ble_freertos__rx_buffer_filler_task(void *parameter)
{
  uint8_t rx_byte;

  while (true)
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
void silabs_ble_freertos__initialize(unsigned long priority)
{
  silabs_ble_freertos__configure_usart();

  usart_rx_queue = xQueueCreate(1024, sizeof(uint8_t));
  usart_rx_buffer_mutex = xSemaphoreCreateMutex();
  ble_mutex = xSemaphoreCreateMutex();
  usart_rx_buffer_not_empty_semphr = xSemaphoreCreateBinary();

  xTaskCreate(silabs_ble_freertos__ble_main_task, "ble test task", 4096 / sizeof(void *), NULL, priority, NULL);
  xTaskCreate(silabs_ble_freertos__rx_buffer_filler_task, "buffer filler task", 2048 / sizeof(void *), NULL, (priority + 1), NULL);
}

bool silabs_ble_freertos__send_notification(uint8_t connection,
                                            uint16_t characteristic_handle,
                                            uint32_t length,
                                            const uint8_t *values)
{
  sl_status_t status;
  xSemaphoreTake(ble_mutex, portMAX_DELAY);
  status = sl_bt_gatt_server_send_notification(connection, characteristic_handle, length, values);
  xSemaphoreGive(ble_mutex);

  if (status != SL_STATUS_OK)
  {
    return false;
  }

  return true;
}

bool silabs_ble_freertos__register_on_event_callback(silabs_ble_freertos__on_event_callback_f on_event_cb)
{
  on_event = on_event_cb;
  return true;
}