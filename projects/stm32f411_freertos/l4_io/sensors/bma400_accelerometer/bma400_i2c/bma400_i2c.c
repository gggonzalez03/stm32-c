#include "bma400_i2c.h"
#include "bma400_i2c_glue.h"

/*************************************************************************
 * @note
 * LSB Sensitivity is calculated by (2^bit_width / (full_scale_rage * 2)) LSB/g
 * Minimum Detection Limit is calculated by (full_scale_range * 2 / (2^bit_width)) g/LSB
 *    Note that this is reciprocal of LSB Sensitivity
 *    Note that the formula could also be rewritten as (full_scale_range / (2^(bit_width - 1)))
 *************************************************************************/

/*************************************************************************
 * 
 *                        PRIVATE DATA DEFINITIONS
 *
 *************************************************************************/
static const uint8_t bit_width = 12;
static const uint8_t g_range = 2;
static const float earth_gravity = 9.80665f;
/*************************************************************************
 * 
 *                      PRIVATE FUNCTION DEFINITIONS
 *
 *************************************************************************/

static void bma400_i2c__normal_mode(bma400_i2c__address bma400_address)
{
  uint8_t acc_config0_reg = 0x19;
  uint8_t normal_mode = 0x02;

  bma400_i2c__write(bma400_address, acc_config0_reg, &normal_mode, 1);
}

static void bma400_i2c__range_2g_osr_high_odr_100Hz(bma400_i2c__address bma400_address)
{
  uint8_t acc_config1_reg = 0x1A;
  uint8_t range_2g = 0x00;
  uint8_t oversampling_rate_highest = 0x03;
  uint8_t output_data_rate_100Hz = 0x08;
  uint8_t all_configs = (range_2g << 6) | (oversampling_rate_highest << 4) | (output_data_rate_100Hz << 0);

  bma400_i2c__write(bma400_address, acc_config1_reg, &all_configs, 1);
}

static void bma400_i2c__fixed_odr_filter_100Hz(bma400_i2c__address bma400_address)
{
  uint8_t acc_config2_reg = 0x1B;
  uint8_t odr_filter_fixed_100Hz = 0x01 << 2;

  bma400_i2c__write(bma400_address, acc_config2_reg, &odr_filter_fixed_100Hz, 1);
}

static void bma400_i2c__enable_data_ready_interrupt(bma400_i2c__address bma400_address)
{
  uint8_t int_config0 = 0x1F;
  uint8_t data_ready_int = 0x80;

  bma400_i2c__write(bma400_address, int_config0, &data_ready_int, 1);
}

static void bma400_i2c__map_data_ready_interrupt_to_int1_pin(bma400_i2c__address bma400_address)
{
  uint8_t int_config1 = 0x21;
  uint8_t data_ready_int1 = 0x80;

  bma400_i2c__write(bma400_address, int_config1, &data_ready_int1, 1);
}

static void bma400_i2c__get_negative(int16_t *value)
{
  if (*value > 2047)
  {
    *value -= 4096;
  }
}

/*************************************************************************
 * 
 *                      PUBLIC FUNCTION DEFINITIONS
 *
 *************************************************************************/
bool bma400_i2c__init(bma400_i2c__address bma400_address)
{
  uint8_t chip_id;
  bma400_i2c__configure();

  chip_id = bma400_i2c__get_chip_id(bma400_address);

  if (chip_id != 0x90)
  {
    return false;
  }

  bma400_i2c__normal_mode(bma400_address);
  bma400_i2c__delay_ms(2);
  bma400_i2c__range_2g_osr_high_odr_100Hz(bma400_address);
  bma400_i2c__fixed_odr_filter_100Hz(bma400_address);
  bma400_i2c__enable_data_ready_interrupt(bma400_address);
  bma400_i2c__map_data_ready_interrupt_to_int1_pin(bma400_address);

  return true;
}

uint8_t bma400_i2c__get_chip_id(bma400_i2c__address bma400_address)
{
  uint8_t chip_id_reg = 0x00;
  uint8_t chip_id;

  bma400_i2c__read(bma400_address, chip_id_reg, &chip_id, 1);

  return chip_id;
}

bma400_i2c__axes_raw_s bma400_i2c__get_acceleration_raw(bma400_i2c__address bma400_address)
{
  const uint8_t x_lsb_reg = 0x04;
  uint8_t xyz_bytes[6];

  bma400_i2c__axes_raw_s xyz_raw;

  bma400_i2c__read(bma400_address, x_lsb_reg, xyz_bytes, 6);

  xyz_raw.x = (int16_t)((xyz_bytes[1] & 0x0F) << 8) | xyz_bytes[0];
  bma400_i2c__get_negative(&xyz_raw.x);
  xyz_raw.y = (int16_t)((xyz_bytes[3] & 0x0F) << 8) | xyz_bytes[2];
  bma400_i2c__get_negative(&xyz_raw.y);
  xyz_raw.z = (int16_t)((xyz_bytes[5] & 0x0F) << 8) | xyz_bytes[4];
  bma400_i2c__get_negative(&xyz_raw.z);

  return xyz_raw;
}

bma400_i2c__axes_mps2_s bma400_i2c__get_acceleration_mps2(bma400_i2c__address bma400_address)
{
  int16_t half_scale;

  bma400_i2c__axes_raw_s xyz_raw = bma400_i2c__get_acceleration_raw(bma400_address);
  bma400_i2c__axes_mps2_s xyz_mps2;

  half_scale = 1 << (bit_width - 1);

  xyz_mps2.x = (earth_gravity * xyz_raw.x * g_range) / half_scale;
  xyz_mps2.y = (earth_gravity * xyz_raw.y * g_range) / half_scale;
  xyz_mps2.z = (earth_gravity * xyz_raw.z * g_range) / half_scale;

  return xyz_mps2;
}

bool bma400_i2c__data_ready(bma400_i2c__address bma400_address)
{
  const uint8_t status_reg = 0x03;
  const uint8_t ready_bit = 1 << 7;
  uint8_t status;

  bma400_i2c__read(bma400_address, status_reg, &status, 1);

  if (status & ready_bit)
  {
    return true;
  }

  return false;
}

int16_t bma400_i2c__get_x_raw(bma400_i2c__address bma400_address)
{
  const uint8_t x_lsb_reg = 0x04;
  uint8_t x_bytes[2];
  int16_t x_raw;

  bma400_i2c__read(bma400_address, x_lsb_reg, &x_bytes, 2);
  x_raw = (int16_t)((x_bytes[1] & 0x0F) << 8) | x_bytes[0];
  bma400_i2c__get_negative(&x_raw);

  return x_raw;
}

int16_t bma400_i2c__get_y_raw(bma400_i2c__address bma400_address)
{
  const uint8_t y_lsb_reg = 0x06;
  uint8_t y_bytes[2];
  int16_t y_raw;

  bma400_i2c__read(bma400_address, y_lsb_reg, &y_bytes, 2);
  y_raw = (int16_t)((y_bytes[1] & 0x0F) << 8) | y_bytes[0];
  bma400_i2c__get_negative(&y_raw);

  return y_raw;
}

int16_t bma400_i2c__get_z_raw(bma400_i2c__address bma400_address)
{
  const uint8_t z_lsb_reg = 0x08;
  uint8_t z_bytes[2];
  int16_t z_raw;

  bma400_i2c__read(bma400_address, z_lsb_reg, &z_bytes, 2);
  z_raw = (int16_t)((z_bytes[1] & 0x0F) << 8) | z_bytes[0];
  bma400_i2c__get_negative(&z_raw);

  return z_raw;
}

float bma400_i2c__get_x_mps2(bma400_i2c__address bma400_address)
{
  int16_t half_scale, x_raw;
  float x_mps2;

  half_scale = 1 << (bit_width - 1);
  x_raw = bma400_i2c__get_x_raw(bma400_address);
  x_mps2 = (earth_gravity * x_raw * g_range) / half_scale;

  return x_mps2;
}

float bma400_i2c__get_y_mps2(bma400_i2c__address bma400_address)
{
  int16_t half_scale, y_raw;
  float y_mps2;

  half_scale = 1 << (bit_width - 1);
  y_raw = bma400_i2c__get_y_raw(bma400_address);
  y_mps2 = (earth_gravity * y_raw * g_range) / half_scale;

  return y_mps2;
}

float bma400_i2c__get_z_mps2(bma400_i2c__address bma400_address)
{
  int16_t half_scale, z_raw;
  float z_mps2;

  half_scale = 1 << (bit_width - 1);
  z_raw = bma400_i2c__get_z_raw(bma400_address);
  z_mps2 = (earth_gravity * z_raw * g_range) / half_scale;

  return z_mps2;
}