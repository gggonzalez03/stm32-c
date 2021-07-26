#pragma once

#include <stdint.h>
#include <stdbool.h>

// Addresses here are already pre-shifted
typedef enum
{
  BMA400_I2C__0X28 = 0x28,
  BMA400_I2C__0X2A = 0x2A
} bma400_i2c__address;

typedef struct
{
  int16_t x, y, z;
} bma400_i2c__axes_raw_s;

typedef struct
{
  float x, y, z;
} bma400_i2c__axes_mps2_s;

/**
 * Initialize bma400
 **/
bool bma400_i2c__init(bma400_i2c__address bma400_address);

/**
 * Get BMA400 chip id
 * @return bma400 chip id
 **/
uint8_t bma400_i2c__get_chip_id(bma400_i2c__address bma400_address);

/**
 * Get raw x, y, and z values
 * @return raw data of x, y, and z axes from bma400
 **/
bma400_i2c__axes_raw_s bma400_i2c__get_acceleration_raw(bma400_i2c__address bma400_address);

/**
 * Get x, y, and z values in meters per second squared (m/s^2 or mps2)
 * @return x, y, and z data in meters per second squared
 **/
bma400_i2c__axes_mps2_s bma400_i2c__get_acceleration_mps2(bma400_i2c__address bma400_address);

/**
 * Check if new data is ready
 * @return true of new data is available
 **/
bool bma400_i2c__data_ready(bma400_i2c__address bma400_address);

/**
 * Get raw x value
 * @return raw x value 
 **/
int16_t bma400_i2c__get_x_raw(bma400_i2c__address bma400_address);

/**
 * Get raw y value
 * @return raw y value 
 **/
int16_t bma400_i2c__get_y_raw(bma400_i2c__address bma400_address);

/**
 * Get raw z value
 * @return raw z value 
 **/
int16_t bma400_i2c__get_z_raw(bma400_i2c__address bma400_address);

/**
 * Get x value in meters per second squared
 * @return x value in mps2
 **/
float bma400_i2c__get_x_mps2(bma400_i2c__address bma400_address);

/**
 * Get y value in meters per second squared
 * @return y value in mps2
 **/
float bma400_i2c__get_y_mps2(bma400_i2c__address bma400_address);

/**
 * Get z value in meters per second squared
 * @return z value in mps2
 **/
float bma400_i2c__get_z_mps2(bma400_i2c__address bma400_address);