#pragma once

#include <stdint.h>

#include "sl_bt_api.h"

void silabs_ble_freertos__configure_usart(void);

void silabs_ble_freertos__uart_transmit_one_byte(uint8_t byte);

void silabs_ble_freertos__on_event(sl_bt_msg_t *evt);