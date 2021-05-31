#include "bma400_spi_glue.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm_peripherals.h"
#include "gpio.h"
#include "spi1.h"

/**
 * TODO:
 * 1. Use a mutex to protect the SPI1 bus
 * 2. Allow multiple chip selects
 **/

static const gpio__gpio_s interrupt1_pin = {GPIO__PORT_A, 3};
static const gpio__gpio_s cs_pin = {GPIO__PORT_A, 4};
static const gpio__gpio_s clk_pin = {GPIO__PORT_A, 5};
static const gpio__gpio_s miso_pin = {GPIO__PORT_A, 6};
static const gpio__gpio_s mosi_pin = {GPIO__PORT_A, 7};

void bma400_spi__configure_spi(uint32_t spi_clock_hz)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_SPI1, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOA, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_SYSCFG, false);

  gpio__set_as_output(cs_pin);
  gpio__configure_with_function(clk_pin.port, clk_pin.pin, GPIO__AF05);
  gpio__configure_with_function(miso_pin.port, miso_pin.pin, GPIO__AF05);
  gpio__configure_with_function(mosi_pin.port, mosi_pin.pin, GPIO__AF05);

  gpio__set_as_input(interrupt1_pin);
  gpio__enable_pull_down_resistor(interrupt1_pin);
  gpio__enable_interrupt(interrupt1_pin, GPIO__RISING_EDGE);

  spi1__init(2 * 1000 * 1000);
}

void bma400_spi__cs(void)
{
  gpio__reset(cs_pin);
}

void bma400_spi__ds(void)
{
  gpio__set(cs_pin);
}

uint8_t bma400_spi__exchange_byte(uint8_t tx_byte, bool is_last_byte)
{
  return spi1__exchange_byte(tx_byte, is_last_byte);
}

void bma400_spi__transmit_receive_bytes(uint8_t *tx_bytes, uint8_t *rx_bytes, uint32_t count)
{
  spi1__transmit_receive_bytes(tx_bytes, rx_bytes, count, true);
}

void bma400_spi__transmit_bytes(uint8_t *tx_bytes, uint32_t count)
{
  spi1__transmit_bytes(tx_bytes, count, true);
}

void bma400_spi__receive_bytes(uint8_t *rx_bytes, uint32_t count)
{
  spi1__receive_bytes(rx_bytes, count, true);
}

void bma400_spi__delay_ms(uint32_t ms)
{
  /**
   * TODO:
   * 1. Use a hardware timer instead if FreeRTOS is not running
   **/
  vTaskDelay(ms);
}