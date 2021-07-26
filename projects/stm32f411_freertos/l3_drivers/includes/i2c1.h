#pragma once

#include <stdint.h>

void i2c1__init(void);
void i2c1__read(uint8_t slave_address, uint8_t register_address, uint8_t* rx_bytes_buffer, uint32_t length);
void i2c1__write(uint8_t slave_address, uint8_t register_address, uint8_t* tx_bytes, uint32_t length);