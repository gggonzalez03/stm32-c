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

static const uint8_t bit_width = 12;
static const uint8_t g_range = 2;
static const float earth_gravity = 9.80665f;

/*************************************************************************
 * 
 *                      PRIVATE FUNCTION DEFINITIONS
 *
 *************************************************************************/
static void bma400_spi__write_to_register(uint8_t reg, uint8_t *tx_bytes, uint32_t count)
{
  bma400_spi__cs();
  (void)bma400_spi__exchange_byte((reg & ~(read_write_mask)), false);
  bma400_spi__transmit_bytes(tx_bytes, count);
  bma400_spi__ds();
}

static void bma400_spi__read_from_register(uint8_t reg, uint8_t *rx_bytes, uint32_t count)
{
  bma400_spi__cs();
  (void)bma400_spi__exchange_byte((reg | read_write_mask), false);
  (void)bma400_spi__exchange_byte(dummy_byte, false);
  bma400_spi__receive_bytes(rx_bytes, count);
  bma400_spi__ds();
}

static void bma400_spi__normal_mode(void)
{
  uint8_t acc_config0_reg = 0x19;
  uint8_t normal_mode = 0x02;

  bma400_spi__write_to_register(acc_config0_reg, &normal_mode, 1);
}

static void bma400_spi__range_2g_osr_high_odr_100Hz(void)
{
  uint8_t acc_config1_reg = 0x1A;
  uint8_t range_2g = 0x00;
  uint8_t oversampling_rate_highest = 0x03;
  uint8_t output_data_rate_100Hz = 0x08;
  uint8_t all_configs = (range_2g << 6) | (oversampling_rate_highest << 4) | (output_data_rate_100Hz << 0);

  bma400_spi__write_to_register(acc_config1_reg, &all_configs, 1);
}

static void bma400_spi__fixed_odr_filter_100Hz(void)
{
  uint8_t acc_config2_reg = 0x1B;
  uint8_t odr_filter_fixed_100Hz = 0x01 << 2;

  bma400_spi__write_to_register(acc_config2_reg, &odr_filter_fixed_100Hz, 1);
}

static void bma_400__enable_data_ready_interrupt(void)
{
  uint8_t int_config0 = 0x1F;
  uint8_t data_ready_int = 0x80;

  bma400_spi__write_to_register(int_config0, &data_ready_int, 1);
}

static void bma_400__map_data_ready_interrupt_to_int1_pin(void)
{
  uint8_t int_config1 = 0x21;
  uint8_t data_ready_int1 = 0x80;

  bma400_spi__write_to_register(int_config1, &data_ready_int1, 1);
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

  bma400_spi__read_from_register(chip_id_reg, &chip_id, 1);

  return chip_id;
}

bool bma400_spi__data_ready(void)
{
  const uint8_t status_reg = 0x03;
  const uint8_t ready_bit = 1 << 7;
  uint8_t status;

  bma400_spi__read_from_register(status_reg, &status, 1);

  if (status & ready_bit)
  {
    return true;
  }

  return false;
}

bma400_spi__axes_raw_s bma400_spi__get_acceleration_raw(void)
{
  const uint8_t x_lsb_reg = 0x04;
  uint8_t xyz_bytes[6];

  bma400_spi__axes_raw_s xyz_raw;

  bma400_spi__read_from_register(x_lsb_reg, xyz_bytes, 6);

  xyz_raw.x = (int16_t)((xyz_bytes[1] & 0x0F) << 8) | xyz_bytes[0];
  bma400_spi__get_negative(&xyz_raw.x);
  xyz_raw.y = (int16_t)((xyz_bytes[3] & 0x0F) << 8) | xyz_bytes[2];
  bma400_spi__get_negative(&xyz_raw.y);
  xyz_raw.z = (int16_t)((xyz_bytes[5] & 0x0F) << 8) | xyz_bytes[4];
  bma400_spi__get_negative(&xyz_raw.z);

  return xyz_raw;
}

bma400_spi__axes_mps2_s bma400_spi__get_acceleration_mps2(void)
{
  int16_t half_scale;

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
  uint8_t x_bytes[2];
  int16_t x_raw;

  bma400_spi__read_from_register(x_lsb_reg, x_bytes, 2);
  x_raw = (int16_t)((x_bytes[1] & 0x0F) << 8) | x_bytes[0];
  bma400_spi__get_negative(&x_raw);

  return x_raw;
}

int16_t bma400_spi__get_y_raw(void)
{
  const uint8_t y_lsb_reg = 0x06;
  uint8_t y_bytes[2];
  int16_t y_raw;

  bma400_spi__read_from_register(y_lsb_reg, y_bytes, 2);
  y_raw = (int16_t)((y_bytes[1] & 0x0F) << 8) | y_bytes[0];
  bma400_spi__get_negative(&y_raw);

  return y_raw;
}
int16_t bma400_spi__get_z_raw(void)
{
  const uint8_t z_lsb_reg = 0x08;
  uint8_t z_bytes[2];
  int16_t z_raw;

  bma400_spi__read_from_register(z_lsb_reg, z_bytes, 2);
  z_raw = (int16_t)((z_bytes[1] & 0x0F) << 8) | z_bytes[0];
  bma400_spi__get_negative(&z_raw);

  return z_raw;
}

float bma400_spi__get_x_mps2(void)
{
  int16_t half_scale, x_raw;
  float x_mps2;

  half_scale = 1 << (bit_width - 1);
  x_raw = bma400_spi__get_x_raw();
  x_mps2 = (earth_gravity * x_raw * g_range) / half_scale;

  return x_mps2;
}

float bma400_spi__get_y_mps2(void)
{
  int16_t half_scale, y_raw;
  float y_mps2;

  half_scale = 1 << (bit_width - 1);
  y_raw = bma400_spi__get_y_raw();
  y_mps2 = (earth_gravity * y_raw * g_range) / half_scale;

  return y_mps2;
}
float bma400_spi__get_z_mps2(void)
{
  int16_t half_scale, z_raw;
  float z_mps2;

  half_scale = 1 << (bit_width - 1);
  z_raw = bma400_spi__get_z_raw();
  z_mps2 = (earth_gravity * z_raw * g_range) / half_scale;

  return z_mps2;
}