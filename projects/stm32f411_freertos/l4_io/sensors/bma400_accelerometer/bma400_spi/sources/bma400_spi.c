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
static const uint8_t read_dummy_count = 1;

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
  const uint8_t acc_config1_reg = 0x1A;
  const uint8_t range_2g = 0x00;
  const uint8_t oversampling_rate_highest = 0x03;
  const uint8_t output_data_rate_100Hz = 0x08;
  const uint8_t all_configs = (range_2g << 6) | (oversampling_rate_highest << 4) | (output_data_rate_100Hz << 0);

  bma400_spi__write_to_register(acc_config1_reg, all_configs);
}

static void bma400_spi__fixed_odr_filter_100Hz(void)
{
  uint8_t acc_config2_reg = 0x1B;
  uint8_t odr_filter_fixed_100Hz = 0x01;

  bma400_spi__write_to_register(acc_config2_reg, odr_filter_fixed_100Hz << 2);
}

static void bma_400__enable_data_ready_interrupt(void)
{
  const uint8_t int_config0 = 0x1F;
  const uint8_t data_ready_int = 0x80;

  bma400_spi__write_to_register(int_config0, data_ready_int);
}

static void bma_400__map_data_ready_interrupt_to_int1_pin(void)
{
  const uint8_t int_config1 = 0x21;
  const uint8_t data_ready_int1 = 0x80;

  bma400_spi__write_to_register(int_config1, data_ready_int1);
}

static void bma400_spi__get_negative(int16_t *value)
{
  if (*value > 2047)
  {
    *value -= 4096;
  }
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
  bma_400__enable_data_ready_interrupt();
  bma_400__map_data_ready_interrupt_to_int1_pin();

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

bma400_spi__axes_raw_s bma400_spi__get_acceleration_raw(void)
{
  const uint8_t x_lsb_reg = 0x04;
  const uint32_t byte_count = 7 + read_dummy_count;

  uint8_t tx_bytes[8] = {x_lsb_reg | read_write_mask};
  uint8_t xyz_bytes[8];

  bma400_spi__axes_raw_s xyz_raw;

  bma400_spi__cs();
  bma400_spi__transmit_receive_bytes(tx_bytes, xyz_bytes, byte_count);
  bma400_spi__ds();

  xyz_raw.x = (int16_t)((xyz_bytes[3] & 0x0F) << 8) | xyz_bytes[2];
  bma400_spi__get_negative(&xyz_raw.x);
  xyz_raw.y = (int16_t)((xyz_bytes[5] & 0x0F) << 8) | xyz_bytes[4];
  bma400_spi__get_negative(&xyz_raw.y);
  xyz_raw.z = (int16_t)((xyz_bytes[7] & 0x0F) << 8) | xyz_bytes[6];
  bma400_spi__get_negative(&xyz_raw.z);

  return xyz_raw;
}

bma400_spi__axes_mps2_s bma400_spi__get_acceleration_mps2(void)
{
  int16_t half_scale;
  const uint8_t bit_width = 12;
  const uint8_t g_range = 2;
  const float earth_gravity = 9.80665f;

  bma400_spi__axes_raw_s xyz_raw = bma400_spi__get_acceleration_raw();
  bma400_spi__axes_mps2_s xyz_mps2;

  half_scale = 1 << (bit_width - 1);

  xyz_mps2.x = (earth_gravity * xyz_raw.x * g_range) / half_scale;
  xyz_mps2.y = (earth_gravity * xyz_raw.y * g_range) / half_scale;
  xyz_mps2.z = (earth_gravity * xyz_raw.z * g_range) / half_scale;

  return xyz_mps2;
}

int16_t bma400_spi__get_x_raw(void)
{
  const uint8_t x_lsb_reg = 0x04;
  const uint32_t byte_count = 8;

  uint8_t tx_bytes[8] = {x_lsb_reg | read_write_mask};
  uint8_t x_bytes[8];

  int16_t x_raw;

  bma400_spi__cs();
  bma400_spi__transmit_receive_bytes(tx_bytes, x_bytes, byte_count);
  bma400_spi__ds();

  x_raw = ((int16_t)x_bytes[3] << 8) | x_bytes[2];
  bma400_spi__get_negative(&x_raw);

  return x_raw;
}

int16_t bma400_spi__get_y_raw(void);
int16_t bma400_spi__get_z_raw(void);

float bma400_spi__get_x_mps2(void)
{
  int16_t half_scale;
  const uint8_t bit_width = 12;
  const uint8_t g_range = 2;
  const float earth_gravity = 9.80665f;

  int16_t x_raw = bma400_spi__get_x_raw();
  float x_mps2;

  half_scale = 1 << (bit_width - 1);

  x_mps2 = (earth_gravity * x_raw * g_range) / 1024.0f;
  // x_mps2 = (float)x_raw / (1024.0f / 2.0f * 9.8f);

  return x_mps2;
}

float bma400_spi__get_y_mps2(void);
float bma400_spi__get_z_mps2(void);