
/**
 ******************************************************************************
 * @file    adxl345.h
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    12-Jan-2025
 * @brief   Contains all the prototypes for the adxl345.C
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 mcu-dev
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef ADXL345_H
#define ADXL345_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "utils/i2c.h"

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} adxl345_axes_data;

/*******************************STATUSES***************************************/
typedef enum {
  ADXL345_STATUS_SUCCESS   = 0,
  ADXL345_STATUS_API_ERR   = -1,
  ADXL345_STATUS_INPUT_ERR = -2,
  ADXL345_STATUS_INIT_ERR  = -3,
} ADXL345_RETURN_STATUS;

/*****************************ID REGISTERS*************************************/

#define ADXL345_I2C_ADDRESS 0x53
#define ADXL345_DEV_ID      0xE5

/*********************************MASKS****************************************/

#define ADXL345_POWER_MODE_MASK    0x04
#define ADXL345_ODR_MASK           0x00
#define ADXL345_SCALE_MASK         0x01
#define ADXL345_RESOLUTION_MASK    0x03
#define ADXL345_MEASURE_MASK       0x03
#define ADXL345_ACTIVITY_EN_MASK   0x04
#define ADXL345_INACTIVITY_EN_MASK 0x00
#define ADXL345_TAP_EN_MASK        0x00

#define ADXL345_INT_DATA_READY 0x07
#define ADXL345_INT_SINGLE_TAP 0x06
#define ADXL345_INT_DOUBLE_TAP 0x05
#define ADXL345_INT_ACTIVITY   0x04
#define ADXL345_INT_INACTIVITY 0x03
#define ADXL345_INT_FREE_FALL  0x02
#define ADXL345_INT_WATERMARK  0x01
#define ADXL345_INT_OVERRUNY   0x00

#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

/*********************************REGISTERS************************************/

#define ADXL345_REG_DEV_ID         0x00
#define ADXL345_REG_THRESH_TAP     0x1D
#define ADXL345_REG_OFFSET_X       0x1E
#define ADXL345_REG_OFFSET_Y       0x1F
#define ADXL345_REG_OFFSET_Z       0x20
#define ADXL345_REG_DUR            0x21
#define ADXL345_REG_LATENT         0x22
#define ADXL345_REG_WINDOW         0x23
#define ADXL345_REG_THRESH_ACT     0x24
#define ADXL345_REG_THRESH_INACT   0x25
#define ADXL345_REG_TIME_INACT     0x26
#define ADXL345_REG_ACT_INACT_CTL  0x27
#define ADXL345_REG_THRESH_FF      0x28
#define ADXL345_REG_TIME_FF        0x29
#define ADXL345_REG_TAP_AXES       0x2A
#define ADXL345_REG_ACT_TAP_STATUS 0x2B
#define ADXL345_REG_BW_RATE        0x2C
#define ADXL345_REG_POWER_CTL      0x2D
#define ADXL345_REG_INT_ENABLE     0x2E
#define ADXL345_REG_INT_MAP        0x2F
#define ADXL345_REG_INT_SOURCE     0x30
#define ADXL345_REG_DATA_FORMAT    0x31
#define ADXL345_REG_DATAX0         0x32
#define ADXL345_REG_DATAX1         0x33
#define ADXL345_REG_DATAY0         0x34
#define ADXL345_REG_DATAY1         0x35
#define ADXL345_REG_DATAZ0         0x36
#define ADXL345_REG_DATAZ1         0x37
#define ADXL345_REG_FIFO_CTL       0x38
#define ADXL345_REG_FIFO_STATUS    0x39

/*****************************ACC DESCRIPTORS**********************************/
typedef enum {
  ADXL345_NORMAL_MODE    = 0x00,
  ADXL345_LOW_POWER_MODE = 0x01
} adxl345_power_mode;

typedef enum {
  ADXL345_STANDBY_MODE = 0x00,
  ADXL345_MEASURE_MODE = 0x01
} adxl345_measure_mode;

typedef enum {
  ADXL345_ODR_0_10HZ = 0x00,
  ADXL345_ODR_0_20HZ = 0x01,
  ADXL345_ODR_0_39HZ = 0x02,
  ADXL345_ODR_0_78HZ = 0x03,
  ADXL345_ODR_1_56HZ = 0x04,
  ADXL345_ODR_3_13HZ = 0x05,
  ADXL345_ODR_6_25HZ = 0x06,
  ADXL345_ODR_12_5HZ = 0x07,
  ADXL345_ODR_25HZ   = 0x08,
  ADXL345_ODR_50HZ   = 0x09,
  ADXL345_ODR_100HZ  = 0x0A,
  ADXL345_ODR_200HZ  = 0x0B,
  ADXL345_ODR_400HZ  = 0x0C,
  ADXL345_ODR_800HZ  = 0x0D,
  ADXL345_ODR_1K6HZ  = 0x0E,
  ADXL345_ODR_3K2HZ  = 0x0F,
} adxl345_odr;

typedef enum {
  ADXL345_SCALE_2G  = 0x00,
  ADXL345_SCALE_4G  = 0x01,
  ADXL345_SCALE_8G  = 0x02,
  ADXL345_SCALE_16G = 0x03,

} adxl345_scale;

typedef enum {
  ADXL345_RES_10BIT = 0x00,
  ADXL345_RES_16BIT = 0x01,
} adxl345_resolution;

typedef enum {
  AXES_DISABLE_ALL = 0x00,
  AXES_ENABLE_X    = 0x01,
  AXES_ENABLE_Y    = 0x02,
  AXES_ENABLE_Z    = 0x04,
  AXES_ENABLE_XY   = 0x03,
  AXES_ENABLE_XZ   = 0x05,
  AXES_ENABLE_YZ   = 0x06,
  AXES_ENABLE_XYZ  = 0x07
} adxl345_axes_enable;

typedef struct {
  bool x;
  bool y;
  bool z;
} axes_status;

typedef struct {
  bool data_ready;
  bool single_tap;
  bool double_tap;
  bool activity;
  bool inactivity;
  bool free_fall;
  bool watermark;
  bool overrun;
} interrupt_status;

typedef struct {
  int8_t x;
  int8_t y;
  int8_t z;
} offset_config;

typedef struct {
  uint8_t threshold;
  uint8_t duration;
  uint8_t latency;
  uint8_t window;
  adxl345_axes_enable tap_en;
  axes_status axes_status;
  axes_status tap_status;
} tap_config;

typedef struct {
  uint8_t activity_threshold;
  uint8_t inactivity_threshold;
  adxl345_axes_enable activity_en;
  adxl345_axes_enable inactivity_en;
  axes_status axes_status;
  axes_status activity_status;
} activity_config;

typedef struct {
  uint8_t threshold;
  uint8_t timeout;
} freefall_config;

/**********************************HANDLES*************************************/

typedef struct {
  adxl345_power_mode power_mode;
  adxl345_resolution resolution;
  adxl345_measure_mode measure;
  adxl345_scale scale;
  adxl345_odr odr;
  interrupt_status interrupt_status;

  activity_config activity_config;
  freefall_config freefall_config;
  offset_config offset_config;
  tap_config tap_config;

  bool is_Setup;
} adxl345_dev;

typedef struct {
  adxl345_power_mode power_mode;
  adxl345_resolution resolution;
  adxl345_measure_mode measure;
  adxl345_scale scale;
  adxl345_odr odr;
  interrupt_status interrupt_status;

  activity_config activity_config;
  freefall_config freefall_config;
  offset_config offset_config;
  tap_config tap_config;

  bool is_Setup;
} adxl345_init_param;

/*******************************PROTOTYPES*************************************/

int8_t adxl345_write_register_value(uint8_t dev_addr, uint8_t *data_buffer);

int8_t adxl345_read_register_value(uint8_t address, uint8_t *val);

int8_t adxl345_setup(adxl345_dev *dev, adxl345_init_param adxl345_params);

bool adxl345_online(void);

int8_t adxl345_set_power_mode(adxl345_dev *device, adxl345_power_mode mode);

int8_t adxl345_set_measure_mode(adxl345_dev *device,
                                adxl345_measure_mode measure);

int8_t adxl345_set_odr(adxl345_dev *device, adxl345_odr odr);

int8_t adxl345_set_scale(adxl345_dev *device, adxl345_scale scale);

int8_t adxl345_set_resolution(adxl345_dev *device,
                              adxl345_resolution resolution);

int8_t adxl345_set_offset_x(adxl345_dev *device, int8_t offset);

int8_t adxl345_set_offset_y(adxl345_dev *device, int8_t offset);

int8_t adxl345_set_offset_z(adxl345_dev *device, int8_t offset);

int8_t adxl345_set_tap_threshold(adxl345_dev *device, uint8_t tap_threshold);

int8_t adxl345_set_tap_duration(adxl345_dev *device, int8_t duration);

int8_t adxl345_set_tap_latency(adxl345_dev *device, int8_t latency);

int8_t adxl345_set_tap_window(adxl345_dev *device, int8_t window);

int8_t adxl345_tap_axes_enable(adxl345_dev *device, adxl345_axes_enable tap_en);

int8_t adxl345_set_activity_threshold(adxl345_dev *device, int8_t threshold);

int8_t adxl345_set_inactivity_threshold(adxl345_dev *device, int8_t threshold);

int8_t adxl345_set_activity_axes_enable(adxl345_dev *device,
                                        adxl345_axes_enable activity_en);

int8_t adxl345_set_inactivity_axes_enable(adxl345_dev *device,
                                          adxl345_axes_enable inactivity_en);

int8_t adxl345_set_freefall_threshold(adxl345_dev *device, int8_t threshold);

int8_t adxl345_set_freefall_timeout(adxl345_dev *device, int8_t timeout);

int8_t adxl345_get_activity_tap_status(adxl345_dev *device);

int8_t adxl345_set_interrupt_enable(adxl345_dev *device, uint8_t interrupt,
                                    int enable);

int8_t adxl345_set_interrupt_map(adxl345_dev *device, uint8_t interrupt,
                                 int map);

int8_t adxl345_get_interrupt_status(adxl345_dev *device);

int8_t adxl345_get_axes_data_x(adxl345_dev *device, adxl345_axes_data *data);

int8_t adxl345_get_axes_data_y(adxl345_dev *device, adxl345_axes_data *data);

int8_t adxl345_get_axes_data_z(adxl345_dev *device, adxl345_axes_data *data);
#endif