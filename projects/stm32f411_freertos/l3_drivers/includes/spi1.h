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
 * Transmit multiple bytes over SPI1
 * @param bytes is the address of the first byte
 * @param count is the number of bytes to transfer
 **/
void spi1__transmit_bytes(uint8_t *bytes, uint32_t count);

/**
 * Transmit and receive bytes over SPI1
 * @param tx_bytes address of the first byte to transmit
 * @param tx_bytes address of the first memory location to store bytes into
 * @param count number of bytes to transfer and receive
 **/
void spi1__transmit_receive_bytes(uint8_t *tx_bytes, uint8_t *rx_bytes, uint32_t count);