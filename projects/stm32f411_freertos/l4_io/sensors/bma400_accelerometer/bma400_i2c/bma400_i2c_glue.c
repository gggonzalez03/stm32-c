#include <stdbool.h>

#include "bma400_i2c_glue.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm_peripherals.h"
#include "gpio.h"
#include "i2c1.h"

void bma400_i2c__configure(void)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOB, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_I2C1, false);
  gpio__configure_with_function(GPIO__PORT_B, 8, GPIO__AF04); // I2C1 SCL
  gpio__configure_with_function(GPIO__PORT_B, 9, GPIO__AF04); // I2C1 SDA

  i2c1__init(false);
}

void bma400_i2c__read(uint8_t slave_address, uint8_t register_address, uint8_t* rx_bytes_buffer, uint32_t length)
{
  i2c1__read(slave_address, register_address, rx_bytes_buffer, length);
}

void bma400_i2c__write(uint8_t slave_address, uint8_t register_address, uint8_t* tx_bytes, uint32_t length)
{
  i2c1__write(slave_address, register_address, tx_bytes, length);
}

void bma400_i2c__delay_ms(uint32_t ms)
{
  /**
   * TODO:
   * 1. Use a hardware timer instead if FreeRTOS is not running
   **/
  vTaskDelay(ms);
}