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

static uint8_t advertising_set_handle = 0xFF;
static uint8_t connection_id = 0xFF;
static void system_id_changed_callback(uint8array *value);
static void silabs_ble_freertos__on_event(sl_bt_msg_t *evt);

void accelerometer_task(void *parameter);
void silabs_ble_freertos__write_characteristic_task(void *parameter);

SemaphoreHandle_t data_ready;
QueueHandle_t z_queue;

int main()
{
  data_ready = xSemaphoreCreateBinary();
  z_queue = xQueueCreate(10, sizeof(uint8_t));
  
  silabs_ble_freertos__register_on_event_callback(silabs_ble_freertos__on_event);
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
  uint8_t z;
  uint32_t length = 1;

  while (1)
  {
    if (connection_id != 0xFF)
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

static void system_id_changed_callback(uint8array *value)
{
  uint8_t i;
  for (i = 0; i < value->len; i++)
  {
    printf("my_data[%d] = 0x%x \r\n", i, value->data[i]);
  }
  printf("--------------------------------\n");
}

static void silabs_ble_freertos__on_event(sl_bt_msg_t *evt)
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
    connection_id = 0xFF;
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