#include <stdio.h>

#include "i2c_test.h"

#include "gpio.h"
#include "stm_peripherals.h"
#include "stm32f411xe.h"
#include "clock.h"

#include "FreeRTOS.h"
#include "task.h"

static void i2c_test__configure(void)
{
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_GPIOB, false);
  stm_peripheral__power_on_peripheral(STM_PERIPHERAL_I2C1, false);
  gpio__configure_with_function(GPIO__PORT_B, 8, GPIO__AF04); // I2C1 SCL
  gpio__configure_with_function(GPIO__PORT_B, 9, GPIO__AF04); // I2C1 SDA

  /**
   * Steps:
   * Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
   * Configure the clock control registers
   * Configure the rise time register
   * Program the I2C_CR1 register to enable the peripheral
   * Set the START bit in the I2C_CR1 register to generate a Start condition
   **/

  I2C1->CR1 |= (1<<15);
	I2C1->CR1 &= ~(1<<15);

  uint32_t frequency_mask = (0x3F << 0);
  I2C1->CR2 &= ~(frequency_mask);
  I2C1->CR2 |= (clock__get_apb1_clock_frq() / 1000 * 1000);

  /**
   * CCR Calculation: Standard Mode
   * 
   * T_high = CCR * T_pclk1   <--- (1)
   * T_high = t_rscl + twsclh <--- (2)
   * T_pclk1 = 1 / 42000000 sec => 23.809524 ns
   * 
   * CCR = t_rscl + twsclh / T_pclk1
   * 
   * For the following values, refer to the datasheet Chapter 6.3.19 Table 58
   * t_rscl = 1000 ns
   * twsclh = 4 us => 4000 ns
   * 
   * CCR = 1000 ns + 4000 ns / 23.809524 ns
   * CCR = 210
   **/
  uint32_t standard_mode = (1 << 15);
  uint32_t duty_cycle_2 = (1 << 14);
  uint32_t ccr_mask = (0x0FFF << 0);
  I2C1->CCR &= ~(ccr_mask);
  I2C1->CCR |= 210;
  I2C1->CCR |= (standard_mode | duty_cycle_2);

  /**
   * TRISE Calculation: Standard Mode
   * 
   * TRISE = (t_rscl / T_pclk1) + 1
   * T_pclk1 = 1 / 42000000 sec => 23.809524 ns
   * 
   * For the following values, refer to the datasheet Chapter 6.3.19 Table 58
   * t_rscl = 1000 ns
   * 
   * TRISE = (1000 ns / 23.809524 ns) + 1
   * TRISE = 42.99 => 42 since TRISE must be programmed with the integer part only according to Chapter 18.6.9
   **/
  uint32_t trise_mask = (0x3F << 0);
  I2C1->TRISE &= ~(trise_mask);
  I2C1->TRISE |= 42;

  // Enable I2C1
  uint32_t i2c1_enable = (1 << 0);
  I2C1->CR1 |= i2c1_enable;
}

static void i2c_test__start(void)
{
  /**
   * Steps:
   * Set the START generation bit
   * Wait for the SB bit to be set
   **/
  uint32_t start_generation = (1 << 8);
  uint32_t start_bit_generated = (1 << 0);
  I2C1->CR1 |= start_generation;

  // Wait for the SB bit to be set
  while ((I2C1->SR1 & start_bit_generated) != start_bit_generated)
  {
    ;
  }
}

static void i2c_test__write_byte(uint8_t slave_address, uint8_t reg,uint8_t byte)
{
  /**
   * Steps:
   * Write slave address to the DR
   * Wait for ADDR bit to be set in the SR1
   * Read SR1 and then SR2
   * --------------------------------------------------------
   * Wait until TxE bit is set (when transmit buffer is empty)
   * Write data byte to the DR
   * Wait until TxE bit is set (when transmit buffer is empty)
   * Write data byte again if there is more
   * Wait for both Txe and BTF bits to set
   * Set the STOP generation bit to close the communication
   **/
}

static void i2c_test__read_byte(uint8_t slave_address, uint8_t reg, uint8_t *byte)
{
  /**
   * Steps:
   * Write slave address to the DR with last bit cleared
   * Wait for ADDR bit to be set in the SR1
   * Read SR1 and then SR2
   * 
   * Wait until TxE bit is set (when transmit buffer is empty)
   * Write data reg to the DR
   * Generate a repeat start
   * 
   * Write slave address to the DR with last bit set
   * Wait for ADDR bit to be set in the SR1
   * Read SR1 and then SR2
   * --------------------------------------------------------
   * If receiving just one byte:
   * Set ACK=0
   * Wait for RxNE=1
   * Read Data from DR (NACK will be automatically sent after this read)
   * Set the STOP generation bit to close the communication
   * --------------------------------------------------------
   * If receiving multiple bytes:
   * Set ACK=1
   * Wait for RxNE=1
   * Read Data 1 from DR (ACK will be automatically sent after this read)
   * Wait for RxNE=1
   * Read Data 2 from DR (ACK will be automatically sent after this read)
   * .
   * .
   * .
   * Set ACK=0
   * Wait for RxNE=1
   * Read Data N from DR (NACK will be automatically sent after this read)
   * Set the STOP generation bit to close the communication
   **/

  uint32_t addr_bit = (1 << 1);
  uint32_t txe_bit = (1 << 7);
  uint32_t btf_bit = (1 << 2);
  uint32_t ack_bit = (1 << 10);
  uint32_t rxne = (1 << 6);
  uint32_t stop_generation = (1 << 9);

  I2C1->DR = slave_address & 0xFE;
  while ((I2C1->SR1 & addr_bit) != addr_bit)
  {
    ;
  }
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  while ((I2C1->SR1 & txe_bit) != txe_bit)
  {
    ;
  }
  I2C1->DR = reg;
  while (((I2C1->SR1 & txe_bit) != txe_bit) && ((I2C1->SR1 & btf_bit) != btf_bit))
  {
    ;
  }

  i2c_test__start();

  I2C1->DR = slave_address;
  while ((I2C1->SR1 & addr_bit) != addr_bit)
  {
    ;
  }
  (void)I2C1->SR1;
  (void)I2C1->SR2;
  
  // --------------------------------------------------------
  
  I2C1->CR1 &= ~(ack_bit);
  while ((I2C1->SR1 & rxne) != rxne)
  {
    ;
  }

  *byte = I2C1->DR;
  I2C1->CR1 |= stop_generation;
}

void i2c_test__task(void *parameter)
{
  uint8_t byte;

  i2c_test__configure();

  while (1)
  {
    i2c_test__start();
    i2c_test__read_byte((0x28 | 0x01), 0x00, &byte);

    printf("BMA400 Chip ID: %d\n", byte);

    vTaskDelay(1000);
  }
}