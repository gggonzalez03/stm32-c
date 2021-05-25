#pragma once

#include <stdint.h>

/**
 * Initialize SPI1
 * @param max_clock_hz is the desired SPI1 clock frequency in Hz
 **/
void spi1__init(uint32_t max_clock_hz);

/**
 * Set SPI1 clock frequency
 * @param max_clock_hz is the desired SPI1 clock frequency in Hz
 **/
void spi1__set_max_clock(uint32_t max_clock_hz);

/**
 * Transmit and receive a byte
 * @param tx_byte is the byte to be transmitted
 * @return rx_byte is the received byte
 **/
uint8_t spi1__exchange_byte(uint8_t tx_byte);

/**
 * Transmit multiple bytes over SPI1
 * @param bytes is the address of the first byte
 * @param count is the number of bytes to transfer
 **/ 
void spi2__transmit_bytes(uint8_t *bytes, uint32_t count);