/**
 ******************************************************************************
 * @file    adxl345.c
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    12-Jan-2025
 * @brief   Contains all the functionalities to control the ADXL345
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

#include "adxl345.h"

/**
 * @brief Write a register value from the ADXL345 accelerometer.
 *
 * @param dev_addr The register address to write the value from.
 * @param data_buffer Pointer to the buffer containing the data to be written.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */

int8_t adxl345_write_register_value(uint8_t dev_addr, uint8_t *data_buffer) {
  return i2c_write_byte(dev_addr, data_buffer);
}

/**
 * @brief Write a multiple values to a register from the ADXL345 accelerometer.
 *
 * @param dev_addr The register address to write the value from.
 * @param data_buffer Pointer to the buffer containing the data to be written.
 * @param bytecount Number of bytes to be sent to the register address.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */

int8_t adxl345_multiple_write_register_value(uint8_t dev_addr,
                                             uint8_t *data_buffer,
                                             uint32_t bytecount) {
  return i2c_write_multiple_bytes(dev_addr, data_buffer, bytecount);
}

/**
 * @brief Read a register value from the ADXL345 accelerometer.
 *
 * @param address The register address to read the value from.
 * @param val Pointer to a variable to store the read register value.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_read_register_value(uint8_t address, uint8_t *val) {
  if (i2c_read_byte(ADXL345_I2C_ADDRESS, address, val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }
  return ADXL345_STATUS_SUCCESS;
}

/**
 * @brief Initializes and sets up the ADXL345 device.
 *
 * Allocates and initializes the device structure and configures it with the
 * specified initialization parameters.
 *
 * @param device        Double pointer to the ADXL345 device structure to be
 * allocated and initialized.
 * @param adxl345_params Initialization parameters for the ADXL345 device.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_setup(adxl345_dev *dev, adxl345_init_param adxl345_params) {
  int8_t ret = 0;

  if (!i2c_init()) {
    return ADXL345_STATUS_INIT_ERR;
  }

  dev->power_mode = adxl345_params.power_mode;
  dev->odr        = adxl345_params.odr;
  dev->resolution = adxl345_params.resolution;
  dev->scale      = adxl345_params.scale;

  ret |= adxl345_set_measure_mode(dev, ADXL345_STANDBY_MODE);
  ret |= adxl345_set_power_mode(dev, dev->power_mode);
  ret |= adxl345_set_odr(dev, dev->odr);
  ret |= adxl345_set_scale(dev, dev->scale);
  ret |= adxl345_set_resolution(dev, dev->resolution);
  ret |= adxl345_set_interrupt_enable(dev, ADXL345_INT_DATA_READY, 1);
  ret |= adxl345_set_interrupt_enable(dev, ADXL345_INT_WATERMARK, 1);
  ret |= adxl345_set_interrupt_enable(dev, ADXL345_INT_OVERRUNY, 1);
  ret |= adxl345_set_measure_mode(dev, ADXL345_MEASURE_MODE);

  if (ret == ADXL345_STATUS_SUCCESS) {
    dev->is_Setup = true;
  } else {
    dev->is_Setup = false;
  }

  return ret;
}

/**
 * @brief Checks if the ADXL345 is online
 *
 * Checks if the ADXL345 is online by checking the read-only register DEV_ID
 *
 * @param None.
 *
 * @return `true` connected if the return value is 0x0E,
 *         `false` otherwise (future implementation).
 */
bool adxl345_online(void) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, 0x00, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }
  if (val != ADXL345_DEV_ID) {
    return false;
  }
  return true;
}

/**
 * @brief Sets the accelerometer power mode.
 *
 * Configures the accelerometer's power mode to optimize power consumption
 * or performance based on the specified mode.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_power_mode(adxl345_dev *device, adxl345_power_mode mode) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_BW_RATE, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x10;
  val = val | mode << ADXL345_POWER_MODE_MASK;

  device->power_mode = mode;

  uint8_t data_buffer[] = {ADXL345_REG_BW_RATE, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Sets the accelerometer measurement mode.
 *
 * Configures the accelerometer's measurement mode
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_measure_mode(adxl345_dev *device,
                                adxl345_measure_mode measure) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_POWER_CTL, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x08;
  val = val | measure << ADXL345_MEASURE_MASK;

  device->measure = measure;

  uint8_t data_buffer[] = {ADXL345_REG_POWER_CTL, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the Output Data Rate (ODR) for the accelerometer.
 *
 * Sets the output data rate of the accelerometer to control how frequently
 * the device outputs data.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_odr(adxl345_dev *device, adxl345_odr odr) {
  uint8_t val = 0x00;

  if (device->power_mode == ADXL345_LOW_POWER_MODE) {
    if (odr > ADXL345_ODR_400HZ) {
      // Refer to Table 8
      return ADXL345_STATUS_INPUT_ERR;
    }
  }

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_BW_RATE, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x0F;
  val = val | odr << ADXL345_ODR_MASK;

  device->odr = odr;

  uint8_t data_buffer[] = {ADXL345_REG_BW_RATE, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the accelerometer scale for the ADXL345 device.
 *
 * Sets the full-scale range of the accelerometer based on the
 * specified scale parameter.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_scale(adxl345_dev *device, adxl345_scale_config scale) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x03;
  val = val | scale.scale << ADXL345_SCALE_MASK;

  device->scale.scale = scale.scale;

  switch (scale.scale) {
  case ADXL345_SCALE_2G:
    device->scale.fs = ADXL345_FULL_SCALE_2G;
    break;
  case ADXL345_SCALE_4G:
    device->scale.fs = ADXL345_FULL_SCALE_4G;
    break;
  case ADXL345_SCALE_8G:
    device->scale.fs = ADXL345_FULL_SCALE_8G;
    break;
  case ADXL345_SCALE_16G:
    device->scale.fs = ADXL345_FULL_SCALE_16G;
    break;
  }

  uint8_t data_buffer[] = {ADXL345_REG_DATA_FORMAT, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the accelerometer resolution for the ADXL345 device.
 *
 * Sets the resolution of the accelerometer based on the specified
 * resolution parameter.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param resolution  Resolution setting for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_resolution(adxl345_dev *device,
                              adxl345_resolution_config resolution) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x08;
  val = val | resolution.resolution << ADXL345_RESOLUTION_MASK;

  device->resolution.resolution = resolution.resolution;
  if (resolution.resolution == ADXL345_RES_10BIT) {
    device->resolution.bits = 10;
    device->resolution.mask = 6;
  } else {
    switch (device->scale.scale) {
    case ADXL345_SCALE_2G:
      device->resolution.bits = 10;
      device->resolution.mask = 6;
      break;
    case ADXL345_SCALE_4G:
      device->resolution.bits = 11;
      device->resolution.mask = 5;
      break;
    case ADXL345_SCALE_8G:
      device->resolution.bits = 12;
      device->resolution.mask = 4;
      break;
    case ADXL345_SCALE_16G:
      device->resolution.bits = 13;
      device->resolution.mask = 3;
      break;
    }
  }

  uint8_t data_buffer[] = {ADXL345_REG_DATA_FORMAT, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap x-axis offset for the ADXL345 accelerometer. Offset
 * scale factor is 15.6mg/LSB
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param offset Offset to be added in the x-axis (-128 to 127)
 *      -128: -128 x 0.0156 g/LSB = -2g
 *         0:    0 x 0.0156 g/LSB =  0g
 *       127:  127 x 0.0156 g/LSB =  2g
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_offset_x(adxl345_dev *device, int8_t offset) {
  device->offset_config.x = offset;

  uint8_t data_buffer[] = {ADXL345_REG_OFFSET_X, offset};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap y-axis offset for the ADXL345 accelerometer. Offset
 * scale factor is 15.6mg/LSB
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param offset Offset to be added in the y-axis (-128 to 127)
 *      -128: -128 x 0.0156 g/LSB = -2g
 *         0:    0 x 0.0156 g/LSB =  0g
 *       127:  127 x 0.0156 g/LSB =  2g
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_offset_y(adxl345_dev *device, int8_t offset) {
  device->offset_config.y = offset;

  uint8_t data_buffer[] = {ADXL345_REG_OFFSET_Y, offset};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap z-axis offset for the ADXL345 accelerometer. Offset
 * scale factor is 15.6mg/LSB
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param offset Offset to be added in the z-axis (-128 to 127)
 *      -128: -128 x 0.0156 g/LSB = -2g
 *         0:    0 x 0.0156 g/LSB =  0g
 *       127:  127 x 0.0156 g/LSB =  2g
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_offset_z(adxl345_dev *device, int8_t offset) {
  device->offset_config.z = offset;

  uint8_t data_buffer[] = {ADXL345_REG_OFFSET_Z, offset};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap threshold for the ADXL345 accelerometer.
 *
 *  Sets the tap threshold for both single and double tap. Tap scale factor
 *  is 62.5mg/LSB
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param threshold The desired tap threshold (0-255)
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_tap_threshold(adxl345_dev *device, uint8_t threshold) {
  device->tap_config.threshold = threshold;

  uint8_t data_buffer[] = {ADXL345_REG_THRESH_TAP, threshold};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap duration for the ADXL345 accelerometer.
 *
 * Configures the maximum time that an event must be above the tap threshold to
 * qualify as a tap. Each LSB represents 625 µs.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param duration The desired tap duration (0-255).
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_tap_duration(adxl345_dev *device, int8_t duration) {
  device->tap_config.duration = duration;

  uint8_t data_buffer[] = {ADXL345_REG_DUR, duration};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap latency for the ADXL345 accelerometer.
 *
 * Configures the wait time between the detection of a single tap and the time
 * window for a double tap to begin. Each LSB represents 1.25 ms.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param latency The desired tap latency (0-255).
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_tap_latency(adxl345_dev *device, int8_t latency) {
  device->tap_config.latency = latency;

  uint8_t data_buffer[] = {ADXL345_REG_LATENT, latency};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap window for the ADXL345 accelerometer.
 *
 * Configures the maximum time that can elapse after the latency period in which
 * a second tap can be detected. Each LSB represents 1.25 ms.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param window The desired tap window (0-255).
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_tap_window(adxl345_dev *device, int8_t window) {
  device->tap_config.window = window;

  uint8_t data_buffer[] = {ADXL345_REG_WINDOW, window};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Enable or disable tap detection on the axes for the ADXL345
 * accelerometer.
 *
 * Configures which axes are enabled for tap detection. This can be configured
 * to detect taps on the X, Y, and/or Z axes by setting the corresponding bits.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param tap_en Bitmask to enable or disable axes for tap detection:
 *               - Bit 0: X-axis
 *               - Bit 1: Y-axis
 *               - Bit 2: Z-axis
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_tap_axes_enable(adxl345_dev *device,
                               adxl345_axes_enable tap_en) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_TAP_AXES, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x07;
  val = val | tap_en << ADXL345_TAP_EN_MASK;

  device->tap_config.tap_en = tap_en;

  uint8_t data_buffer[] = {ADXL345_REG_TAP_AXES, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the activity threshold for the ADXL345 accelerometer.
 *
 * Configures the activity threshold to determine the minimum acceleration
 * needed to qualify as activity. Each LSB represents 62.5 mg.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param threshold The desired activity threshold (0-255).
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_activity_threshold(adxl345_dev *device, int8_t threshold) {
  device->activity_config.activity_threshold = threshold;

  uint8_t data_buffer[] = {ADXL345_REG_THRESH_ACT, threshold};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the inactivity threshold for the ADXL345 accelerometer.
 *
 * Configures the inactivity threshold to determine the minimum acceleration
 * needed to qualify as inactivity. Each LSB represents 62.5 mg.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param threshold The desired inactivity threshold (0-255).
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_inactivity_threshold(adxl345_dev *device, int8_t threshold) {
  device->activity_config.inactivity_threshold = threshold;

  uint8_t data_buffer[] = {ADXL345_REG_THRESH_INACT, threshold};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Enable or disable activity detection on the axes for the ADXL345
 * accelerometer.
 *
 * Configures which axes are enabled for activity detection. This can be
 * configured to detect activity on the X, Y, and/or Z axes by setting the
 * corresponding bits.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param activity_en Bitmask to enable or disable axes for activity detection:
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_activity_axes_enable(adxl345_dev *device,
                                        adxl345_axes_enable activity_en) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_INACT_CTL, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x70;
  val = val | activity_en << ADXL345_ACTIVITY_EN_MASK;

  device->activity_config.activity_en = activity_en;

  uint8_t data_buffer[] = {ADXL345_REG_ACT_INACT_CTL, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Enable or disable inactivity detection on the axes for the ADXL345
 * accelerometer.
 *
 * Configures which axes are enabled for inactivity detection. This can be
 * configured to detect inactivity on the X, Y, and/or Z axes by setting the
 * corresponding bits.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param inactivity_en Bitmask to enable or disable axes for inactivity
 * detection:
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_inactivity_axes_enable(adxl345_dev *device,
                                          adxl345_axes_enable inactivity_en) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_INACT_CTL, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x07;
  val = val | inactivity_en << ADXL345_INACTIVITY_EN_MASK;

  device->activity_config.inactivity_en = inactivity_en;

  uint8_t data_buffer[] = {ADXL345_REG_ACT_INACT_CTL, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the free-fall threshold for the ADXL345 accelerometer.
 *
 * Configures the acceleration threshold to detect free-fall events.
 * Each LSB represents 62.5 mg.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param threshold The desired free-fall threshold (0-255).
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_freefall_threshold(adxl345_dev *device, int8_t threshold) {
  device->freefall_config.threshold = threshold;

  uint8_t data_buffer[] = {ADXL345_REG_THRESH_FF, threshold};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the free-fall timeout for the ADXL345 accelerometer.
 *
 * Configures the amount of time that acceleration must remain below the
 * free-fall threshold to register a free-fall event. Each LSB represents 5 ms.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param timeout The desired free-fall timeout (0-255).
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_freefall_timeout(adxl345_dev *device, int8_t timeout) {
  device->freefall_config.timeout = timeout;

  uint8_t data_buffer[] = {ADXL345_REG_TIME_FF, timeout};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Get the activity and tap status from the ADXL345 accelerometer.
 *
 * Reads the activity and tap status registers and updates the device structure
 * with the current status of activity and tap detection on the X, Y, and Z
 * axes.
 *
 * @param device Pointer to the ADXL345 device structure.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_get_activity_tap_status(adxl345_dev *device) {
  uint8_t val = 0x00;
  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_TAP_STATUS, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  device->activity_config.activity_status.x = (bool)(val & (1 << 6));
  device->activity_config.activity_status.y = (bool)(val & (1 << 5));
  device->activity_config.activity_status.z = (bool)(val & (1 << 4));

  device->tap_config.tap_status.x = (bool)(val & (1 << 2));
  device->tap_config.tap_status.y = (bool)(val & (1 << 1));
  device->tap_config.tap_status.z = (bool)(val & 1);

  return ADXL345_STATUS_SUCCESS;
}

/**
 * @brief Enable or disable specific interrupts on the ADXL345 accelerometer.
 *
 * Configures the interrupt enable register to enable or disable specific
 * interrupts.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param interrupt The interrupt to configure (0-7).
 * @param enable Set to 1 to enable the interrupt or 0 to disable it.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_interrupt_enable(adxl345_dev *device, uint8_t interrupt,
                                    int enable) {
  uint8_t val = 0x00;
  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_ENABLE, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val = val | enable << interrupt;

  uint8_t data_buffer[] = {ADXL345_REG_INT_ENABLE, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Map an interrupt to a specific pin for the ADXL345 accelerometer.
 *
 * Configures the interrupt map register to route a specific interrupt to
 * either INT1 or INT2 pin.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param interrupt The interrupt to configure (0-7).
 * @param map Set to 1 to map the interrupt to INT2 or 0 to map it to INT1.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_interrupt_map(adxl345_dev *device, uint8_t interrupt,
                                 int map) {
  uint8_t val = 0x00;
  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_MAP, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val = val | map << interrupt;

  uint8_t data_buffer[] = {ADXL345_REG_INT_MAP, val};
  return i2c_write_byte(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Get the interrupt status from the ADXL345 accelerometer.
 *
 * Reads the interrupt source register and updates the device structure with
 * the current status of active interrupts.
 *
 * @param device Pointer to the ADXL345 device structure.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_get_interrupt_status(adxl345_dev *device) {
  uint8_t val = 0x00;
  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_SOURCE, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  device->interrupt_status.data_ready = (bool)(val & (1 << 7));
  device->interrupt_status.single_tap = (bool)(val & (1 << 6));
  device->interrupt_status.double_tap = (bool)(val & (1 << 5));
  device->interrupt_status.activity   = (bool)(val & (1 << 4));
  device->interrupt_status.inactivity = (bool)(val & (1 << 3));
  device->interrupt_status.free_fall  = (bool)(val & (1 << 2));
  device->interrupt_status.watermark  = (bool)(val & (1 << 1));
  device->interrupt_status.overrun    = (bool)(val & 1);

  return ADXL345_STATUS_SUCCESS;
}

/**
 * @brief Get the raw X, Y, and Z-axis acceleration data from the ADXL345
 * accelerometer.
 *
 * Reads the acceleration data registers for the X, Y, and Z axes and updates
 * the raw data in the provided `adxl345_axes_data` structure. The data is
 * right-shifted according to the device's resolution setting.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param data Pointer to the structure where the raw X, Y, and Z-axis data will
 * be stored.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_get_raw_xyz(adxl345_dev *device, adxl345_axes_data *data) {
  uint8_t val[6];

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAX1, &val[1]) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAX0, &val[0]) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAY1, &val[3]) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAY0, &val[2]) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAZ1, &val[5]) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAZ0, &val[4]) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  data->raw_data.x = ((val[1] << 8) | val[0]) >> device->resolution.mask;
  data->raw_data.y = ((val[3] << 8) | val[2]) >> device->resolution.mask;
  data->raw_data.z = ((val[5] << 8) | val[4]) >> device->resolution.mask;

  data->raw_data.x = twos_complement(data->raw_data.x, device->resolution.bits);
  data->raw_data.y = twos_complement(data->raw_data.y, device->resolution.bits);
  data->raw_data.z = twos_complement(data->raw_data.z, device->resolution.bits);

  return ADXL345_STATUS_SUCCESS;
}

/**
 * @brief Get the X, Y, and Z-axis acceleration values in g from the ADXL345
 * accelerometer.
 *
 * Reads the acceleration data registers for the X, Y, and Z axes, converts them
 * from raw two's complement format, and updates the provided
 * `adxl345_axes_data` structure. The values are scaled to represent
 * acceleration in g.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param data Pointer to the structure where the X, Y, and Z-axis acceleration
 * values (in g) will be stored.
 */
void adxl345_get_acc_xyz(adxl345_dev *device, adxl345_axes_data *data) {

  float full_scale_range = device->scale.fs / 1.0f;

  data->acc_data.x = (float)(data->raw_data.x * full_scale_range) /
                     (float)(1 << device->resolution.bits);
  data->acc_data.y = (float)(data->raw_data.y * full_scale_range) /
                     (float)(1 << device->resolution.bits);
  data->acc_data.z = (float)(data->raw_data.z * full_scale_range) /
                     (float)(1 << device->resolution.bits);
}

/**
 * @brief Convert a raw unsigned value to a signed two's complement value.
 *
 * This function converts an unsigned integer into a signed integer
 * using the two's complement representation based on the specified bit width.
 *
 * @param value The raw unsigned value.
 * @param bits The number of bits used to represent the value.
 *
 * @return The signed two's complement value.
 */
int16_t twos_complement(uint16_t value, int bits) {
  uint16_t mask = (1 << bits) - 1;

  if (value & (1 << (bits - 1))) {
    return -(int16_t)((~value & mask) + 1);
  } else {
    return (int16_t)value;
  }
}