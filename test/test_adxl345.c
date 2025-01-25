#include "stdint.h"

#include "cmock.h"
#include "unity.h"

#include "adxl345.h"
#include "mock_i2c.h"

static adxl345_dev dev;
static adxl345_init_param init_param;
adxl345_axes_data adxl345_data;

void setUp(void) {
  init_param.power_mode            = ADXL345_NORMAL_MODE;
  init_param.odr                   = ADXL345_ODR_200HZ;
  init_param.scale.scale           = ADXL345_SCALE_2G;
  init_param.resolution.resolution = ADXL345_RES_FULL;
}

void test_adxl345_setup(void) {
  uint8_t measure_standby = 0x00;
  uint8_t power_mode      = 0x00;
  uint8_t odr             = 0x00;
  uint8_t scale           = 0x00;
  uint8_t resolution      = 0x20;
  uint8_t int_dataready   = 0x00;
  uint8_t int_watermark   = 0x80;
  uint8_t int_overrun     = 0x82;
  uint8_t measure_normal  = 0x00;

  uint8_t measure_standby_ret[] = {ADXL345_REG_POWER_CTL, 0x00};
  uint8_t power_mode_ret[]      = {ADXL345_REG_BW_RATE, 0x00};
  uint8_t odr_ret[]             = {ADXL345_REG_BW_RATE, 0x0F};
  uint8_t scale_ret[]           = {ADXL345_REG_DATA_FORMAT, 0x00};
  uint8_t resolution_ret[]      = {ADXL345_REG_DATA_FORMAT, 0x01};
  uint8_t int_drdy_ret[]        = {ADXL345_REG_INT_ENABLE, 0x80};
  uint8_t int_wtrmrk_ret[]      = {ADXL345_REG_INT_ENABLE, 0x82};
  uint8_t int_ovrrn_ret[]       = {ADXL345_REG_INT_ENABLE, 0x83};
  uint8_t measure_normal_ret[]  = {ADXL345_REG_POWER_CTL, 0x01};

  i2c_init_ExpectAndReturn(true);

  // Put ADXL345 to stand-by mode
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_POWER_CTL,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&measure_standby);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, measure_standby_ret,
                                 ADXL345_STATUS_SUCCESS);

  // Mock set power mode
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_BW_RATE, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&power_mode);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, power_mode_ret,
                                 ADXL345_STATUS_SUCCESS);

  // Mock set odr
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_BW_RATE, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&odr);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, odr_ret,
                                 ADXL345_STATUS_SUCCESS);

  // Mock set scale
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&scale);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, scale_ret,
                                 ADXL345_STATUS_SUCCESS);

  // Mock set scale
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&resolution);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, resolution_ret,
                                 ADXL345_STATUS_SUCCESS);

  // Mock set data ready interrupt
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_ENABLE,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&int_dataready);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, int_drdy_ret,
                                 ADXL345_STATUS_SUCCESS);

  // Mock set watermark interrupt
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_ENABLE,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&int_watermark);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, int_wtrmrk_ret,
                                 ADXL345_STATUS_SUCCESS);

  // Mock set overrun interrupt
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_ENABLE,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&int_overrun);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, int_ovrrn_ret,
                                 ADXL345_STATUS_SUCCESS);

  // Put ADXL345 to measure mode
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_POWER_CTL,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&measure_normal);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, measure_normal_ret,
                                 ADXL345_STATUS_SUCCESS);

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS, adxl345_setup(&dev, init_param));
  TEST_ASSERT_EQUAL(ADXL345_NORMAL_MODE, dev.power_mode);
  TEST_ASSERT_EQUAL(ADXL345_ODR_200HZ, dev.odr);
  TEST_ASSERT_EQUAL(ADXL345_SCALE_2G, dev.scale.scale);
  TEST_ASSERT_EQUAL(ADXL345_RES_FULL, dev.resolution.resolution);
  TEST_ASSERT_TRUE(dev.is_Setup);
}

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
  scale.scale = ADXL345_SCALE_2G;

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS, adxl345_set_scale(&dev, scale));
  TEST_ASSERT_EQUAL(dev.scale.scale, ADXL345_SCALE_2G);
  TEST_ASSERT_EQUAL(dev.scale.fs, ADXL345_FULL_SCALE_2G);
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
  TEST_ASSERT_EQUAL(dev.resolution.bits, 10);
  TEST_ASSERT_EQUAL(dev.resolution.mask, 6);
}

void test_adxl345_set_tap_threshold(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_tap_threshold(&dev, 0x32));
  TEST_ASSERT_EQUAL(dev.tap_config.threshold, 0x32);
  // 62.5mg per increment
}

void test_adxl345_set_tap_duration(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_tap_duration(&dev, 0xF));
  TEST_ASSERT_EQUAL(dev.tap_config.duration, 0xF);
  // 625us per increment
}

void test_adxl345_set_tap_latency(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_tap_latency(&dev, 0x50));
  TEST_ASSERT_EQUAL(dev.tap_config.latency, 0x50);
  // 1.25ms per increment
}

void test_adxl345_set_tap_window(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS, adxl345_set_tap_window(&dev, 0xC8));
  TEST_ASSERT_EQUAL(dev.tap_config.window, 0xC8);
  // 1.25ms per increment
}

void test_adxl345_tap_axes_enable(void) {
  uint8_t read_data_result = 0x00;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_TAP_AXES, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_tap_axes_enable(&dev, AXES_ENABLE_XYZ));
  TEST_ASSERT_EQUAL(dev.tap_config.tap_en, AXES_ENABLE_XYZ);
}

void test_adxl345_set_activity_threshold(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_activity_threshold(&dev, 0xF));
  TEST_ASSERT_EQUAL(dev.activity_config.activity_threshold, 0xF);
}

void test_adxl345_set_inactivity_threshold(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_inactivity_threshold(&dev, 0xF));
  TEST_ASSERT_EQUAL(dev.activity_config.inactivity_threshold, 0xF);
}

void test_adxl345_set_activity_axes_enable(void) {
  uint8_t read_data_result = 0x00;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_INACT_CTL,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_activity_axes_enable(&dev, AXES_ENABLE_XYZ));
  TEST_ASSERT_EQUAL(dev.activity_config.activity_en, AXES_ENABLE_XYZ);
}

void test_adxl345_set_inactivity_axes_enable(void) {
  uint8_t read_data_result = 0x00;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_INACT_CTL,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_inactivity_axes_enable(&dev, AXES_ENABLE_XYZ));
  TEST_ASSERT_EQUAL(dev.activity_config.inactivity_en, AXES_ENABLE_XYZ);
}

void test_adxl345_set_freefall_threshold(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_freefall_threshold(&dev, 0xF));
  TEST_ASSERT_EQUAL(dev.freefall_config.threshold, 0xF);
}

void test_adxl345_set_freefall_timeout(void) {
  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_freefall_timeout(&dev, 0xF));
  TEST_ASSERT_EQUAL(dev.freefall_config.timeout, 0xF);
}

void test_adxl345_get_activity_tap_status(void) {
  uint8_t read_data_result = 0x01;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_TAP_STATUS,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_get_activity_tap_status(&dev));

  TEST_ASSERT_FALSE(dev.activity_config.activity_status.x);
  TEST_ASSERT_FALSE(dev.activity_config.activity_status.y);
  TEST_ASSERT_FALSE(dev.activity_config.activity_status.z);

  TEST_ASSERT_FALSE(dev.tap_config.tap_status.x);
  TEST_ASSERT_FALSE(dev.tap_config.tap_status.y);
  TEST_ASSERT_TRUE(dev.tap_config.tap_status.z);
}

void test_adxl345_set_interrupt_enable(void) {
  uint8_t read_data_result = 0x00;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_ENABLE,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(
      ADXL345_STATUS_SUCCESS,
      adxl345_set_interrupt_enable(&dev, ADXL345_INT_DATA_READY, 1));
}

void test_adxl345_set_interrupt_map(void) {
  uint8_t read_data_result = 0x00;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_MAP, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, NULL,
                                 ADXL345_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_set_interrupt_map(&dev, ADXL345_INT_DATA_READY,
                                              ADXL345_INT1_PIN));
}

void test_adxl345_get_interrupt_status(void) {
  uint8_t read_data_result = 0x83;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_INT_SOURCE,
                                NULL, ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS, adxl345_get_interrupt_status(&dev));

  TEST_ASSERT_TRUE(dev.interrupt_status.data_ready);
  TEST_ASSERT_FALSE(dev.interrupt_status.single_tap);
  TEST_ASSERT_FALSE(dev.interrupt_status.double_tap);
  TEST_ASSERT_FALSE(dev.interrupt_status.activity);

  TEST_ASSERT_FALSE(dev.interrupt_status.inactivity);
  TEST_ASSERT_FALSE(dev.interrupt_status.free_fall);
  TEST_ASSERT_TRUE(dev.interrupt_status.watermark);
  TEST_ASSERT_TRUE(dev.interrupt_status.overrun);
}

void test_adxl345_get_raw_xyz(void) {
  uint8_t val[6];
  val[0] = 0x40; // X LSB
  val[1] = 0xFC; // X MSB
  val[2] = 0xFD; // Y LSB
  val[3] = 0xFD; // Y MSB
  val[4] = 0x80; // Z LSB
  val[5] = 0x3A; // Z MSB

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAX1, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val[1]);

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAX0, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val[0]);

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAY1, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val[3]);

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAY0, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val[2]);

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAZ1, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val[5]);

  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DATAZ0, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val[4]);

  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS,
                    adxl345_get_raw_xyz(&dev, &adxl345_data));
  TEST_ASSERT_EQUAL(adxl345_data.raw_data.x, -15);
  TEST_ASSERT_EQUAL(adxl345_data.raw_data.y, -9);
  TEST_ASSERT_EQUAL(adxl345_data.raw_data.z, 234);
}

void test_adxl345_get_acc_xyz(void) {
  adxl345_get_acc_xyz(&dev, &adxl345_data);
  TEST_ASSERT_FLOAT_WITHIN(0.01, -0.06, adxl345_data.acc_data.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01, -0.04, adxl345_data.acc_data.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0.91, adxl345_data.acc_data.z);
}