#include "stdint.h"

#include "cmock.h"
#include "unity.h"

#include "adxl345.h"
#include "mock_i2c.h"

static adxl345_dev dev;
static adxl345_init_param init_param;

void test_adxl345_setup(void) {}

void test_adxl345_online(void) {
  uint8_t read_data_result = 0xE5;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DEV_ID, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  TEST_ASSERT_TRUE(adxl345_online());
  TEST_ASSERT_EQUAL(read_data_result, ADXL345_DEV_ID);
}

void test_adxl345_set_power_mode(void) {
  uint8_t read_data_result = 0x0A;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_BW_RATE, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                  ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_power_mode(&dev, ADXL345_NORMAL_MODE));
  TEST_ASSERT_EQUAL(dev.power_mode, ADXL345_NORMAL_MODE);
}

void test_adxl345_set_measure_mode(void) {
  uint8_t read_data_result = 0x00;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_POWER_CTL,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                  ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_measure_mode(&dev, ADXL345_MEASURE_MODE));
  TEST_ASSERT_EQUAL(dev.measure, ADXL345_MEASURE_MODE);
}

void test_adxl345_set_odr_sucessful(void) {
  uint8_t read_data_result = 0x0A;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_BW_RATE, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                  ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_odr(&dev, ADXL345_ODR_200HZ));
  TEST_ASSERT_EQUAL(dev.odr, ADXL345_ODR_200HZ);
}

void test_adxl345_set_odr_invalid_odr(void) {
  adxl345_power_mode power_mode_t = dev.power_mode;
  dev.power_mode                  = ADXL345_LOW_POWER_MODE;
  TEST_ASSERT_EQUAL(ADXL345_STATUS_INPUT_ERR,
                    adxl345_set_odr(&dev, ADXL345_ODR_800HZ));
  TEST_ASSERT_NOT_EQUAL(dev.odr, ADXL345_ODR_800HZ);
  dev.power_mode = power_mode_t;
}

void test_adxl345_set_scale(void) {
  uint8_t read_data_result = 0x00;
  adxl345_scale_config scale;
  scale.scale = ADXL345_SCALE_8G;

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                  ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS, adxl345_set_scale(&dev, scale));
  TEST_ASSERT_EQUAL(dev.scale.scale, ADXL345_SCALE_8G);
  TEST_ASSERT_EQUAL(dev.scale.fs, ADXL345_FULL_SCALE_8G);
}

void test_adxl345_set_resolution(void) {
  uint8_t read_data_result = 0x00;
  adxl345_resolution_config resolution;
  resolution.resolution = ADXL345_RES_FULL;

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                  ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_resolution(&dev, resolution));
  TEST_ASSERT_EQUAL(dev.resolution.resolution, ADXL345_RES_FULL);
  TEST_ASSERT_EQUAL(dev.resolution.bits, 12);
  TEST_ASSERT_EQUAL(dev.resolution.mask, 4);
}

void test_adxl345_set_tap_threshold(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                  ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_tap_threshold(&dev, 0x10));
  TEST_ASSERT_EQUAL(dev.tap_config.threshold, 0x10);
}
