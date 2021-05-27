#include <stddef.h>

#include "bma400_spi.h"
#include "bma400_spi_glue.h"

static const uint8_t read_write_mask = 1 << 7;
static const uint8_t dummy_byte = 0xFF;

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

  return true;
}

uint8_t bma400_spi__get_chip_id(void)
{
  const uint8_t chip_id_reg = 0x00;
  uint8_t chip_id;

  bma400_spi__cs();
  (void)bma400_spi__exchange_byte((chip_id_reg | read_write_mask), false);
  (void)bma400_spi__exchange_byte(dummy_byte, false);
  chip_id = bma400_spi__exchange_byte(dummy_byte, true);
  bma400_spi__ds();

  return chip_id;
}

bma400_spi_axes_raw_s bma400_spi__get_acceleration_raw(void);
bma400_spi_axes_mps2_s bma400_spi__get_acceleration_mps2(void);
int16_t bma400_spi__get_x_raw(void);
int16_t bma400_spi__get_y_raw(void);
int16_t bma400_spi__get_z_raw(void);
int16_t bma400_spi__get_x_mps2(void);
int16_t bma400_spi__get_y_mps2(void);
int16_t bma400_spi__get_z_mps2(void);