#include <stddef.h>

#include "bma400_spi.h"
#include "bma400_spi_glue.h"

/*************************************************************************
 * 
 *                        PRIVATE DATA DEFINITIONS
 *
 *************************************************************************/
static const uint8_t read_write_mask = 1 << 7;
static const uint8_t dummy_byte = 0xFF;

/*************************************************************************
 * 
 *                      PRIVATE FUNCTION DEFINITIONS
 *
 *************************************************************************/
static void bma400_spi__write_to_register(uint8_t reg, uint8_t value)
{
  bma400_spi__cs();
  (void)bma400_spi__exchange_byte((reg & ~(read_write_mask)), false);
  (void)bma400_spi__exchange_byte(value, true);
  bma400_spi__ds();
}

static uint8_t bma400_spi__read_from_register(uint8_t reg)
{
  uint8_t value;

  bma400_spi__cs();
  (void)bma400_spi__exchange_byte((reg | read_write_mask), false);
  (void)bma400_spi__exchange_byte(dummy_byte, false);
  value = bma400_spi__exchange_byte(dummy_byte, true);
  bma400_spi__ds();

  return value;
}

static void bma400_spi__normal_mode(void)
{
  const uint8_t acc_config0_reg = 0x19;
  const uint8_t normal_mode = 0x02;

  bma400_spi__write_to_register(acc_config0_reg, normal_mode);
}

static void bma400_spi__range_2g_osr_high_odr_100Hz(void)
{
  const uint8_t acc_config1_reg = 0x19;
  const uint8_t range_2g = 0x00;
  const uint8_t oversampling_rate_highest = 0x03;
  const uint8_t output_data_rate_100Hz = 0x08;
  const uint8_t all_configs = range_2g | oversampling_rate_highest | output_data_rate_100Hz;

  bma400_spi__write_to_register(acc_config1_reg, all_configs);
}

static void bma400_spi__fixed_odr_filter_100Hz(void)
{
  uint8_t acc_config2_reg = 0x1B;
  uint8_t odr_filter_fixed_100Hz = 0x04;

  bma400_spi__write_to_register(acc_config2_reg, odr_filter_fixed_100Hz);
}

/*************************************************************************
 * 
 *                        PUBLIC DATA DEFINITIONS
 *
 *************************************************************************/

/*************************************************************************
 * 
 *                      PUBLIC FUNCTION DEFINITIONS
 *
 *************************************************************************/
bool bma400_spi__init(void)
{
  uint8_t chip_id;

  bma400_spi__configure_spi(2 * 1000 * 1000);

  (void)bma400_spi__get_chip_id();
  chip_id = bma400_spi__get_chip_id();

  if (chip_id != 0x90)
  {
    return false;
  }

  /**
   * TODO:
   * Do actual bma400 initialization sequence here
   **/
  bma400_spi__normal_mode();
  bma400_spi__delay_ms(2);
  bma400_spi__range_2g_osr_high_odr_100Hz();
  bma400_spi__fixed_odr_filter_100Hz();

  return true;
}

uint8_t bma400_spi__get_chip_id(void)
{
  const uint8_t chip_id_reg = 0x00;
  uint8_t chip_id;

  chip_id = bma400_spi__read_from_register(chip_id_reg);

  return chip_id;
}

bool bma400_spi__data_ready(void)
{
  const uint8_t status_reg = 0x03;
  const uint8_t ready_bit = 1 << 7;
  uint8_t status;

  status = bma400_spi__read_from_register(status_reg);

  if (status & ~(ready_bit))
  {
    return true;
  }

  return false;
}

bma400_spi__axes_raw_s bma400_spi__get_acceleration_raw(void);
bma400_spi__axes_mps2_s bma400_spi__get_acceleration_mps2(void);
int16_t bma400_spi__get_x_raw(void);
int16_t bma400_spi__get_y_raw(void);
int16_t bma400_spi__get_z_raw(void);
float bma400_spi__get_x_mps2(void);
float bma400_spi__get_y_mps2(void);
float bma400_spi__get_z_mps2(void);