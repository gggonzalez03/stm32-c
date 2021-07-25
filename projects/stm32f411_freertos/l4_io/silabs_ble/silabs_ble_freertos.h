#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "sl_bt_api.h"

typedef void (*silabs_ble_freertos__on_event_callback_f)(sl_bt_msg_t *event);

typedef struct
{
  uint16_t length;
  uint8_t bytes[1024];
} silabs_ble_freertos__buffer_t;

/**
 * Initialize the bluetooth API, tasks and UART connection
 * 
 * Create related FreeRTOS tasks, and configure UART connection
 * 
 * @param priority ble_main_task priority.
 * 
 * @note rx_buffer_filler_task will be ble_main_task priority + 1
 **/
void silabs_ble_freertos__initialize(unsigned long priority);

bool silabs_ble_freertos__send_notification(uint8_t connection,
                                            uint16_t characteristic_handle,
                                            uint8_t length,
                                            const uint8_t *values);

bool silabs_ble_freertos__register_on_event_callback(silabs_ble_freertos__on_event_callback_f on_event_cb);

void silabs_ble_freertos__assert(sl_status_t status);