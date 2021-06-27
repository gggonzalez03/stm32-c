#pragma once

#include <stdint.h>
#include <stdbool.h>

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

/**
 * Get the current connection ID
 * @return true if connection exists. The connection id stored in id
 **/
bool silabs_ble_freertos__get_connection(uint8_t *id);

bool silabs_ble_freertos__send_notification(uint8_t connection,
                                            uint16_t characteristic_handle,
                                            uint32_t length,
                                            const uint8_t *values);