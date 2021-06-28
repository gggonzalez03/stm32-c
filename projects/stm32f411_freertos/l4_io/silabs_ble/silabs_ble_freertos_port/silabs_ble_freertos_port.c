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
extern uint8_t advertising_set_handle;
extern uint8_t connection_id;

void silabs_ble_freertos__configure_usart(void)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_USART2, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);

  usart__init(USART__6, clock__get_core_clock_frq(), 115200, false);
  usart__enable_rx_interrrupt(USART__6);

  gpio__configure_with_function(GPIO__PORT_A, 11, GPIO__AF08); // tx pin
  gpio__configure_with_function(GPIO__PORT_A, 12, GPIO__AF08); // rx pin
}

void silabs_ble_freertos__uart_transmit_one_byte(uint8_t byte)
{
  usart__polled_transmit(USART__6, byte);
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

void system_id_changed_callback(uint8array *value)
{
  uint8_t i;
  for (i = 0; i < value->len; i++)
  {
    printf("my_data[%d] = 0x%x \r\n", i, value->data[i]);
  }
  printf("--------------------------------\n");
}

void silabs_ble_freertos__on_event(sl_bt_msg_t *evt)
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