#include "i2c1.h"

#include "stm32f411xe.h"
#include "clock.h"

/*************************************************************************
 * 
 *                      PRIVATE FUNCTION DEFINITIONS
 *
 *************************************************************************/
static void i2c1__software_reset(void)
{
  uint32_t software_reset_bit = (1 << 15);
  I2C1->CR1 |= software_reset_bit;
  I2C1->CR1 &= ~(software_reset_bit);
}

static void i2c1__set_fast_mode(void)
{
  uint32_t fast_mode = (1 << 15);
  I2C1->CCR |= fast_mode;
}

static void i2c1__set_standard_mode(void)
{
  uint32_t standard_mode = ~(1 << 15);
  I2C1->CCR &= (standard_mode);
}

static void i2c1__set_clock_frequency(void)
{
  uint32_t frequency_mask = (0x3F << 0);
  I2C1->CR2 &= ~(frequency_mask);
  I2C1->CR2 |= (clock__get_apb1_clock_frq() / 1000 * 1000);
}

static void i2c1__config_clock_control_standard_mode(void)
{
  /**
   * CCR Calculation: Standard Mode
   * 
   * Given:
   * For the following values, refer to the datasheet Chapter 6.3.19 Table 58
   * t_rscl = 1000 ns
   * twsclh = 4 us => 4000 ns
   * 
   * T_pclk1 = 1 / 42000000 Hz => 23.809524 ns
   * 
   * Formula:
   * T_high = CCR * T_pclk1   <--- (1)
   * T_high = t_rscl + twsclh <--- (2)
   * 
   * Derivation:
   * CCR = t_rscl + twsclh / T_pclk1
   * 
   * Solution:
   * CCR = 1000 ns + 4000 ns / 23.809524 ns
   * CCR = 210
   **/
  uint32_t ccr_mask = ~(0x0FFF << 0);

  i2c1__set_standard_mode();

  I2C1->CCR &= (ccr_mask);
  I2C1->CCR |= (210 & ~(ccr_mask));
}

static void i2c1__config_clock_control_fast_mode(void)
{
  /**
   * CCR Calculation: Fast Mode
   * 
   * Given:
   * For the following values, refer to the datasheet Chapter 6.3.19 Table 58
   * t_rscl = 300 ns
   * twsclh = 0.6 us => 600 ns
   * 
   * T_pclk1 = 1 / 42000000 Hz => 23.809524 ns
   * 
   * Formula:
   * T_high = CCR * T_pclk1   <--- (1)
   * T_high = t_rscl + twsclh <--- (2)
   * 
   * Derivation:
   * CCR = t_rscl + twsclh / T_pclk1
   * 
   * Solution:
   * CCR = 300 ns + 600 ns / 23.809524 ns
   * CCR = 50
   **/
  uint32_t duty_cycle_2 = (1 << 14);
  uint32_t ccr_mask = ~(0x0FFF << 0);

  i2c1__set_fast_mode();

  I2C1->CCR &= (ccr_mask);
  I2C1->CCR |= ((50 & ~(ccr_mask)) | duty_cycle_2);
}

static void i2c1__config_trise_standard_mode(void)
{
  /**
   * TRISE Calculation: Standard Mode
   * 
   * Given:
   * For the following values, refer to the datasheet Chapter 6.3.19 Table 58
   * t_rscl = 1000 ns
   * T_pclk1 = 1 / 42000000 sec => 23.809524 ns
   * 
   * Formula:
   * TRISE = (t_rscl / T_pclk1) + 1
   * 
   * Solution:
   * TRISE = (1000 ns / 23.809524 ns) + 1
   * TRISE = 42.99 => 42 since TRISE must be programmed with the integer part only according to Chapter 18.6.9
   **/
  uint32_t trise_mask = (0x3F << 0);
  I2C1->TRISE &= ~(trise_mask);
  I2C1->TRISE |= 42;
}

static void i2c1__config_trise_fast_mode(void)
{
  /**
   * TRISE Calculation: Fast Mode
   * 
   * Given:
   * For the following values, refer to the datasheet Chapter 6.3.19 Table 58
   * t_rscl = 300 ns
   * T_pclk1 = 1 / 42000000 sec => 23.809524 ns
   * 
   * Formula:
   * TRISE = (t_rscl / T_pclk1) + 1
   * 
   * Solution:
   * TRISE = (300 ns / 23.809524 ns) + 1
   * TRISE = 12.59 => 12 since TRISE must be programmed with the integer part only according to Chapter 18.6.9
   **/
  uint32_t trise_mask = (0x3F << 0);
  I2C1->TRISE &= ~(trise_mask);
  I2C1->TRISE |= 12;
}

static void i2c1__enable(void)
{
  const uint32_t i2c1_enable = (1 << 0);
  I2C1->CR1 |= i2c1_enable;
}

static void i2c1__generate_start_condition(void)
{
  /**
   * Steps:
   * 1. Set the START generation bit
   * 2. Wait for the SB bit to be set
   **/
  const uint32_t start_generation = (1 << 8);
  const uint32_t start_bit_generated = (1 << 0);

  I2C1->CR1 |= start_generation;

  while ((I2C1->SR1 & start_bit_generated) != start_bit_generated)
  {
    ;
  }
}

static void i2c1__generate_stop_condition(void)
{
  const uint32_t stop_generation = (1 << 9);
  I2C1->CR1 |= stop_generation;
}

static void i2c1__transmit_slave_address(uint8_t slave_address)
{
  const uint32_t addr_bit = (1 << 1);

  I2C1->DR = slave_address;
  while ((I2C1->SR1 & addr_bit) != addr_bit)
  {
    ;
  }
  (void)I2C1->SR1;
  (void)I2C1->SR2;
}

static void i2c1__wait_transmission_completion(void)
{
  const uint32_t txe_bit = (1 << 7);
  const uint32_t btf_bit = (1 << 2);
  // Wait for the byte transfer to finish (BTF == Byte Transfer Finished)
  while (((I2C1->SR1 & txe_bit) != txe_bit) && ((I2C1->SR1 & btf_bit) != btf_bit))
  {
    ;
  }
}

static void i2c1__receive_data_byte(uint8_t *rx_byte)
{
  const uint32_t rxne = (1 << 6);

  while ((I2C1->SR1 & rxne) != rxne)
  {
    ;
  }

  *rx_byte = I2C1->DR;
}

static void i2c1__transmit_data_byte(uint8_t tx_byte)
{
  const uint32_t txe_bit = (1 << 7);

  while ((I2C1->SR1 & txe_bit) != txe_bit)
  {
    ;
  }

  I2C1->DR = tx_byte;
}

static void i2c1__transmit_register_address(uint8_t register_address)
{
  i2c1__transmit_data_byte(register_address);
  i2c1__wait_transmission_completion();
}

/*************************************************************************
 * 
 *                      PUBLIC FUNCTION DEFINITIONS
 *
 *************************************************************************/
void i2c1__init(bool fast_mode)
{
  /**
   * Steps:
   * 1. Software reset
   * 2. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
   * 3. Configure the clock control registers
   * 4. Configure the rise time register
   * 5. Program the I2C_CR1 register to enable the peripheral
   **/

  i2c1__software_reset();
  i2c1__set_clock_frequency();

  if (fast_mode)
  {
    i2c1__config_clock_control_fast_mode();
    i2c1__config_trise_fast_mode();
  }
  else
  {
    i2c1__config_clock_control_standard_mode();
    i2c1__config_trise_standard_mode();
  }

  i2c1__enable();
}

void i2c1__read(uint8_t slave_address, uint8_t register_address, uint8_t *rx_bytes_buffer, uint32_t length)
{
  /**
   * Steps:
   * Generate start condition
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

  const uint32_t ack_bit = (1 << 10);

  uint32_t data_index = 0;

  i2c1__generate_start_condition();
  i2c1__transmit_slave_address(slave_address & 0xFE);
  i2c1__transmit_register_address(register_address);

  // Repeat start
  i2c1__generate_start_condition();
  i2c1__transmit_slave_address(slave_address | 0x01);

  for (data_index = 0; data_index < (length - 1); data_index++)
  {
    I2C1->CR1 |= (ack_bit);
    i2c1__receive_data_byte(&rx_bytes_buffer[data_index]);
  }

  I2C1->CR1 &= ~(ack_bit);
  i2c1__receive_data_byte(&rx_bytes_buffer[data_index]);

  i2c1__generate_stop_condition();
}

void i2c1__write(uint8_t slave_address, uint8_t register_address, uint8_t *tx_bytes, uint32_t length)
{
  /**
   * Steps:
   * Generate start condition
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

  uint32_t data_index = 0;

  i2c1__generate_start_condition();
  i2c1__transmit_slave_address(slave_address & 0xFE);
  i2c1__transmit_register_address(register_address);

  for (data_index = 0; data_index < length; data_index++)
  {
    i2c1__transmit_data_byte(tx_bytes[data_index]);
  }
  i2c1__wait_transmission_completion();
}