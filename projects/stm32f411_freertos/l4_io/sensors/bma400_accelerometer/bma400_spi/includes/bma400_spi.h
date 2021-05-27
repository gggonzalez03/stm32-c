#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct
{

} bma400_spi_s;

typedef struct
{
  int16_t x, y, z;
} bma400_spi_axes_raw_s;

typedef struct
{
  float x, y, z;
} bma400_spi_axes_mps2_s;

/**
 * Initialize bma400
 **/
bool bma400_spi__init(void);

/**
 * Get BMA400 chip id
 * @return bma400 chip id
 **/ 
uint8_t bma400_spi__get_chip_id(void);

/**
 * Get raw x, y, and z values
 * @return raw data of x, y, and z axes from bma400
 **/
bma400_spi_axes_raw_s bma400_spi__get_acceleration_raw(void);

/**
 * Get x, y, and z values in meters per second squared (m/s^2 or mps2)
 * @return x, y, and z data in meters per second squared
 **/
bma400_spi_axes_mps2_s bma400_spi__get_acceleration_mps2(void);

/**
 * Get raw x value
 * @return raw x value 
 **/ 
int16_t bma400_spi__get_x_raw(void);

/**
 * Get raw y value
 * @return raw y value 
 **/ 
int16_t bma400_spi__get_y_raw(void);

/**
 * Get raw z value
 * @return raw z value 
 **/ 
int16_t bma400_spi__get_z_raw(void);

/**
 * Get x value in meters per second squared
 * @return x value in mps2
 **/ 
int16_t bma400_spi__get_x_mps2(void);

/**
 * Get y value in meters per second squared
 * @return y value in mps2
 **/ 
int16_t bma400_spi__get_y_mps2(void);

/**
 * Get z value in meters per second squared
 * @return z value in mps2
 **/ 
int16_t bma400_spi__get_z_mps2(void);
