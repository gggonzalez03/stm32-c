#pragma once

#include <stdint.h>
#include <stdbool.h>

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
 * Exchange single byte. Transmit one and receive one.
 * @param tx_byte the byte to transmit
 * @param spi_off_after_tx whether or not spi should be turned off to save power after transmission
 * @return received byte
 **/
uint8_t spi1__exchange_byte(uint8_t tx_byte, bool spi_off_after_tx);

/**
 * Transmit multiple bytes over SPI1
 * @param tx_bytes is the address of the first byte
 * @param count is the number of bytes to transfer
 * @param spi_off_after_tx whether or not spi should be turned off to save power after transmission
 **/
void spi1__transmit_bytes(uint8_t *tx_bytes, uint32_t count, bool spi_off_after_tx);

/**
 * Receive multiple bytes over SPI1
 * @param bytes is the address of the first location in the rx buffer
 * @param count is the number of bytes to receive
 * @param spi_off_after_tx whether or not spi should be turned off to save power after transmission
 **/
void spi1__receive_bytes(uint8_t *rx_bytes, uint32_t count, bool spi_off_after_tx);

/**
 * Transmit and receive bytes over SPI1
 * @param tx_bytes address of the first byte to transmit
 * @param tx_bytes address of the first memory location to store bytes into
 * @param count number of bytes to transfer and receive
 * @param spi_off_after_tx whether or not spi should be turned off to save power after transmission
 **/
void spi1__transmit_receive_bytes(uint8_t *tx_bytes, uint8_t *rx_bytes, uint32_t count, bool spi_off_after_tx);