#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * Configure the SPI peripheral
 * @param spi_clock_hz spi clock frequency in Hertz
 **/ 
void bma400_spi__configure_spi(uint32_t spi_clock_hz);
void bma400_spi__cs(void);
void bma400_spi__ds(void);

/**
 * Exchange single byte. Transmit one and receive one.
 * @param tx_byte the byte to transmit
 * @param is_last_byte whether the byte to transmit is the last byte or not
 * @return received byte
 **/
uint8_t bma400_spi__exchange_byte(uint8_t tx_byte, bool is_last_byte);

/**
 * Exchange multiple bytes. Transmit n bytes and receive n bytes.
 * @param tx_bytes pointer to the first byte of a buffer
 * @param rx_bytes pointer to the first memory location of a buffer
 * @param count number of bytes
 **/
void bma400_spi__transmit_receive_bytes(uint8_t *tx_bytes, uint8_t *rx_bytes, uint32_t count);

void bma400_spi__delay_ms(uint32_t ms);