#include <stdio.h>

#include "i2c_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bma400_i2c.h"

void i2c_test__task(void *parameter)
{
  uint8_t chip_id;
  bma400_i2c__axes_mps2_s xyz;
  bma400_i2c__init(BMA400_I2C__0X28);

  while (1)
  {
    xyz = bma400_i2c__get_acceleration_mps2(BMA400_I2C__0X28);
    printf("x: %f, y: %f, z: %f\n", xyz.x, xyz.y, xyz.z);
    vTaskDelay(500);
  }
}