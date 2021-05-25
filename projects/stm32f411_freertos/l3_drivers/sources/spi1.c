#include "spi1.h"

#include "stm32f411xe.h"
#include "clock.h"

/*************************************************************************
 * 
 *                        PRIVATE DATA DEFINITIONS
 *
 *************************************************************************/

/*************************************************************************
 * 
 *                      PRIVATE FUNCTION DEFINITIONS
 *
 *************************************************************************/

static void spi1__wait_while_busy(void)
{
  const uint32_t spi_busy = (1UL << 7);

  /**
   * First line waits for the SPI bus to be busy
   * Second line waits for the SPI bus to not be busy
   * The first line is necessary because it takes some time before the busy flag is set after setting the DR
   * register. This is most likely because value of DR register has to be copied into a shadow register before
   * transmitting it.
   **/
  while ((SPI1->SR & spi_busy) != spi_busy)
    ;
  while ((SPI1->SR & spi_busy) == spi_busy)
    ;
}

static void spi1__wait_until_tx_buffer_empty(void)
{
  const uint32_t spi_transmit_buffer_empty = (1UL << 1);
  while ((SPI1->SR & spi_transmit_buffer_empty) != spi_transmit_buffer_empty)
    ;
}

static void spi1__wait_while_rx_buffer_empty(void)
{
  const uint32_t spi_receive_buffer_not_empty = (1UL << 0);
  while ((SPI1->SR & spi_receive_buffer_not_empty) != spi_receive_buffer_not_empty)
    ;
}

static void spi1__enable(void)
{
  const uint32_t spi_enable = (1UL << 6);

  if ((SPI1->CR1 & spi_enable) != spi_enable)
  {
    SPI1->CR1 |= spi_enable;
  }
}

static void spi1__disable(void)
{
  const uint32_t spi_enable = (1UL << 6);

  if ((SPI1->CR1 & spi_enable) == spi_enable)
  {
    SPI1->CR1 &= ~(spi_enable);
  }
}

/*************************************************************************
 * 
 *                        PRIVATE DATA DEFINITIONS
 *
 *************************************************************************/

/*************************************************************************
 * 
 *                        PUBLIC DATA DEFINITIONS
 *
 *************************************************************************/

void spi1__init(uint32_t max_clock_hz)
{
  // Steps followed are outlined in the Reference Manual
  // Chapter 20.3.3
  spi1__set_max_clock(max_clock_hz);

  uint32_t spi_8_bit_data_frame = (1UL << 11);
  uint32_t spi_master_mode = (1UL << 2);
  uint32_t spi_software_slave_mgnt = (1UL << 9);
  uint32_t spi_bi_directional_mode = (1UL << 15);

  // This is set in conjunction with the software slave management bit. See here why SSOE has to be set:
  // https://electronics.stackexchange.com/questions/442794/why-ss-pin-must-be-set-to-output-for-spi-master
  uint32_t spi_slave_select_output_enable = (1UL << 2);

  SPI1->CR1 &= ~(spi_8_bit_data_frame | spi_bi_directional_mode);
  SPI1->CR2 |= spi_slave_select_output_enable;
  SPI1->CR1 |= (spi_master_mode | spi_software_slave_mgnt);
  spi1__enable();
}

void spi1__set_max_clock(uint32_t max_clock_hz)
{
  const uint32_t core_clock = clock__get_core_clock_frq();
  const uint32_t spi_baud_divider_mask = (7UL << 3);

  uint32_t spi_baud_divider = 2UL;
  uint32_t baud_rate_control_reg_val = 0UL;

  while ((core_clock / spi_baud_divider) > max_clock_hz)
  {
    spi_baud_divider = spi_baud_divider << 1;
    baud_rate_control_reg_val++;
  }

  baud_rate_control_reg_val = baud_rate_control_reg_val << 3;

  SPI1->CR1 &= ~(spi_baud_divider_mask);
  SPI1->CR1 |= baud_rate_control_reg_val;
}

uint8_t spi1__exchange_byte(uint8_t tx_byte)
{
  spi1__wait_until_tx_buffer_empty();

  SPI1->DR = tx_byte & 0xFF;

  spi1__wait_while_busy();

  spi1__wait_while_rx_buffer_empty();

  return (uint8_t)SPI1->DR & 0xFF;
}

void spi1__transmit_bytes(uint8_t *bytes, uint32_t count)
{
  for (uint32_t current_byte = 0; current_byte < count; current_byte++)
  {
    spi1__wait_until_tx_buffer_empty();
    SPI1->DR = bytes[current_byte] & 0xFF;
  }

  spi1__wait_while_busy();
}