#pragma once

#include <stdint.h>

void bma400_i2c__configure(void);
void bma400_i2c__read(uint8_t slave_address, uint8_t register_address, uint8_t* rx_bytes_buffer, uint32_t length);
void bma400_i2c__write(uint8_t slave_address, uint8_t register_address, uint8_t* tx_bytes, uint32_t length);
void bma400_i2c__delay_ms(uint32_t ms);