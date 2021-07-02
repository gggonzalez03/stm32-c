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
static void attribute_changed_callback(uint8array *value);
static void silabs_ble_freertos__on_event(sl_bt_msg_t *evt);

void accelerometer_task(void *parameter);
void silabs_ble_freertos__write_characteristic_task(void *parameter);

SemaphoreHandle_t data_ready;
QueueHandle_t z_queue;


typedef struct
{
  uint8_t payload[252];
} ble_payload_t;

int main()
{
  data_ready = xSemaphoreCreateBinary();
  z_queue = xQueueCreate(10, sizeof(ble_payload_t *));

  silabs_ble_freertos__register_on_event_callback(silabs_ble_freertos__on_event);
  silabs_ble_freertos__initialize(4);

  xTaskCreate(silabs_ble_freertos__write_characteristic_task, "write task", 4096 / sizeof(void *), NULL, 1, NULL);
  xTaskCreate(accelerometer_task, "accel task", 4096 / sizeof(void *), NULL, 1, NULL);

  vTaskStartScheduler();

  return 0;
}

void accelerometer_task(void *parameter)
{
  float z;

  ble_payload_t payload;
  uint8_t index_counter = 0;

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
      payload.payload[index_counter] = (uint8_t)z;

      PRINTF("z: %f\n", z);

      xQueueSend(z_queue, (void *)&payload, portMAX_DELAY);
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
  ble_payload_t z;
  uint32_t length = sizeof(ble_payload_t);

  while (1)
  {
    if (connection_id != 0xFF)
    {
      xQueueReceive(z_queue, &z, portMAX_DELAY);
      silabs_ble_freertos__send_notification(connection_id, characteristics[2].handle, length, z.payload);
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

void EXTI0_IRQHandler(void)
{
  xSemaphoreGiveFromISR(data_ready, NULL);
  exti__clear_external_interrupt(0);
}

static void attribute_changed_callback(uint8array *value)
{
  uint8_t i;
  for (i = 0; i < value->len; i++)
  {
    PRINTF("data[%d] = 0x%x \r\n", i, value->data[i]);
  }
  PRINTF("--------------------------------\n");
}

static void silabs_ble_freertos__on_event(sl_bt_msg_t *evt)
{
  sl_status_t ble_status;
  bd_addr address;
  uint8_t address_type;

  switch (SL_BT_MSG_ID(evt->header))
  {
  case sl_bt_evt_system_boot_id:
    ble_status = sl_bt_system_get_identity_address(&address, &address_type);
    silabs_ble_freertos__assert(ble_status);

    // Pad and reverse unique ID to get System ID.
    characteristics[SYSTEM_ID].value[0] = address.addr[5];
    characteristics[SYSTEM_ID].value[1] = address.addr[4];
    characteristics[SYSTEM_ID].value[2] = address.addr[3];
    characteristics[SYSTEM_ID].value[3] = 0xFF;
    characteristics[SYSTEM_ID].value[4] = 0xFE;
    characteristics[SYSTEM_ID].value[5] = address.addr[2];
    characteristics[SYSTEM_ID].value[6] = address.addr[1];
    characteristics[SYSTEM_ID].value[7] = address.addr[0];

    ble_status = gatt_database__initialize();
    silabs_ble_freertos__assert(ble_status);

    ble_status = sl_bt_advertiser_create_set(&advertising_set_handle);
    silabs_ble_freertos__assert(ble_status);

    // Set advertising interval to 100ms.
    ble_status = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
    silabs_ble_freertos__assert(ble_status);

    ble_status = sl_bt_advertiser_start(advertising_set_handle,
                                        sl_bt_advertiser_general_discoverable,
                                        sl_bt_advertiser_connectable_scannable);
    silabs_ble_freertos__assert(ble_status);

    PRINTF("Bluetooth stack booted: v%d.%d.%d-b%d\n",
           evt->data.evt_system_boot.major,
           evt->data.evt_system_boot.minor,
           evt->data.evt_system_boot.patch,
           evt->data.evt_system_boot.build);
    PRINTF("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
           address_type ? "static random" : "public device",
           address.addr[5],
           address.addr[4],
           address.addr[3],
           address.addr[2],
           address.addr[1],
           address.addr[0]);
    PRINTF("Started advertising.\n");

    break;
  case sl_bt_evt_connection_opened_id:
    connection_id = evt->data.evt_connection_opened.connection;
    PRINTF("Connection opened.\n");
    break;
  case sl_bt_evt_connection_closed_id:
    connection_id = 0xFF;
    // Restart advertising after client has disconnected.
    ble_status = sl_bt_advertiser_start(advertising_set_handle,
                                        sl_bt_advertiser_general_discoverable,
                                        sl_bt_advertiser_connectable_scannable);
    silabs_ble_freertos__assert(ble_status);

    PRINTF("Started advertising after disconnect.\n");
    break;
  case sl_bt_evt_gatt_server_attribute_value_id:

    if (characteristics[MANUFACTURER_NAME_STRING].handle == evt->data.evt_gatt_server_attribute_value.attribute)
    {
      attribute_changed_callback(&(evt->data.evt_gatt_server_attribute_value.value));
    }
    if (characteristics[SYSTEM_ID].handle == evt->data.evt_gatt_server_attribute_value.attribute)
    {
      attribute_changed_callback(&(evt->data.evt_gatt_server_attribute_value.value));
    }
    break;
  case sl_bt_evt_gatt_server_characteristic_status_id:
    // PRINTF("Characteristic Event Code: %lx\n", SL_BT_MSG_ID(evt->header));
    break;

  default:
    break;
  }
}