#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/hal.h"
#include "lis2dh12_reg.h"  // ST platform-independent driver

namespace esphome {
namespace lis2dh12 {

// ST driver return code
static const int32_t ST_SUCCESS = 0;

struct LIS2DH12Config {
  bool low_power_mode;
  bool temperature_sensor_state;
  bool accel_x_sensor_state;
  bool accel_y_sensor_state;
  bool accel_z_sensor_state;
  bool block_data_update;
  bool high_resolution_mode;
  //LIS2DH12DataRate data_rate;
  //LIS2DH12FullScale full_scale;
};

enum StateCode : uint8_t {
  READY_OK = 0,
  SEARCHING_FOR_DEVICE,
  RESETTING_DEVICE,
  CONFIGURING_DEVICE,
  BUSY,
  ERROR,
  ERROR_TIMEOUT,
  ERROR_NOT_FOUND,
  ERROR_NOT_RESPONDING,
  ERROR_NOT_CONFIGURED
};

enum DataRate {
  POWER_DOWN                      = lis2dh12_odr_t::LIS2DH12_POWER_DOWN,
  ODR_1HZ                         = lis2dh12_odr_t::LIS2DH12_ODR_1Hz,
  ODR_10HZ                        = lis2dh12_odr_t::LIS2DH12_ODR_10Hz,
  ODR_25HZ                        = lis2dh12_odr_t::LIS2DH12_ODR_25Hz,
  ODR_50HZ                        = lis2dh12_odr_t::LIS2DH12_ODR_50Hz,
  ODR_100HZ                       = lis2dh12_odr_t::LIS2DH12_ODR_100Hz,
  ODR_200HZ                       = lis2dh12_odr_t::LIS2DH12_ODR_200Hz,
  ODR_400HZ                       = lis2dh12_odr_t::LIS2DH12_ODR_400Hz,
  ODR_1KHZ620_LP                  = lis2dh12_odr_t::LIS2DH12_ODR_1kHz620_LP,
  ODR_5KHZ376_LP_1KHZ344_NM_HP    = lis2dh12_odr_t::LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP
};

enum FullScaleRange {
  FS_2G   = lis2dh12_fs_t::LIS2DH12_2g,
  FS_4G   = lis2dh12_fs_t::LIS2DH12_4g,
  FS_8G   = lis2dh12_fs_t::LIS2DH12_8g,
  FS_16G  = lis2dh12_fs_t::LIS2DH12_16g
};

/**
 * @brief Interface for LIS2DH12 event listeners.
 */
class LIS2DH12Listener {
  public:
    /**
     * @brief Called when a click event is detected.
     * 
     * @param detected True if a click is detected, false otherwise.
     */
    virtual void on_click(bool detected) {};

    /**
     * @brief Called when a new X-axis acceleration value is available.
     * 
     * @param xAccel The X-axis acceleration value in m/s^2.
     */
    virtual void on_x_accel_value(float xAccel) {};

    /**
     * @brief Called when a new Y-axis acceleration value is available.
     * 
     * @param yAccel The Y-axis acceleration value in m/s^2.
     */
    virtual void on_y_accel_value(float yAccel) {};

    /**
     * @brief Called when a new Z-axis acceleration value is available.
     * 
     * @param zAccel The Z-axis acceleration value in m/s^2.
     */
    virtual void on_z_accel_value(float zAccel) {};

    /**
     * @brief Called when a new temperature value is available.
     * 
     * @param temp_celsius The temperature value in degrees Celsius.
     */
    virtual void on_temperature_value(float temp_celsius) {};
};

class LIS2DH12Component : public Component, public i2c::I2CDevice {
  public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::HARDWARE; };

  protected:

    StateCode state_ = StateCode::ERROR_NOT_FOUND;


    struct AccelerationSensorConfigT {
      uint8_t x_en : 1;
      uint8_t y_en : 1;
      uint8_t z_en : 1;
      lis2dh12_fs_t full_scale_range;
    };

    struct ClickDetectionAxisConfigT {
      uint8_t x_en : 1;
      uint8_t y_en : 1;
      uint8_t z_en : 1;
      uint8_t negative_only : 1;
    };

    struct TimingConfigT {
      uint8_t time_limit;
      uint8_t time_window;
      uint8_t time_latency;
    };

    struct ClickConfigT {
      lis2dh12_lir_click_t interrupt_mode;
      uint8_t threshold;
      uint8_t double_en : 1;
      ClickDetectionAxisConfigT axis;
      TimingConfigT timing;
    };

    struct SensorConfigT {
      AccelerationSensorConfigT accel;
      lis2dh12_temp_en_t temperature_en;
      lis2dh12_odr_t data_rate;
    };

    struct ConfigT {
      uint8_t block_data_update : 1;
      lis2dh12_op_md_t operation_mode;
      SensorConfigT sensor;
      ClickConfigT click;
    };

    ConfigT current_config_ = {
      .block_data_update = PROPERTY_ENABLE,
      .operation_mode = LIS2DH12_HR_12bit,
      .sensor = {
          .accel = {
              .x_en = PROPERTY_ENABLE,
              .y_en = PROPERTY_ENABLE,
              .z_en = PROPERTY_ENABLE,
              .full_scale_range = LIS2DH12_2g
          },
          .temperature_en = LIS2DH12_TEMP_DISABLE,
          .data_rate = LIS2DH12_ODR_400Hz
      },
      .click = {
          .interrupt_mode = LIS2DH12_TAP_LATCHED,
          .threshold = 0,
          .double_en = PROPERTY_DISABLE,
          .axis = {
              .x_en = PROPERTY_DISABLE,
              .y_en = PROPERTY_DISABLE,
              .z_en = PROPERTY_DISABLE,
              .negative_only = PROPERTY_DISABLE
          },
          .timing = {
              .time_limit = 0,
              .time_window = 0,
              .time_latency = 0
          }
      }
    };

    ConfigT new_config_= current_config_;

    void apply_device_configuration(ConfigT config = new_config_);

    inline void set_error_not_configured_and_mark_failed() {
      this->state_ = ERROR_NOT_CONFIGURED;
      this->mark_failed();
    }

    /**
     * @brief Checks if the LIS2DH12 device is present and the device ID matches the expected value.
     * 
     * This function reads the device ID of the LIS2DH12 sensor and compares it with the expected device ID.
     * If the device ID does not match the expected value, an error message is logged and false is returned.
     * 
     * @return True if the LIS2DH12 device is present and the device ID matches the expected value, false otherwise.
     */
    bool is_device_present();

    /**
     * @brief Resets the LIS2DH12 device.
     * 
     * This function triggers a reset of the LIS2DH12 device by setting the boot 
     * property and waits for the reset to complete within the specified timeout
     * or at least once.
     * 
     * @param timeout_ms The maximum time in milliseconds to wait for the reset to complete.
     * @return True if the reset is successful within the timeout, false otherwise.
     */
    bool reset_device(uint8_t timeout_ms = 100);

    /**
     * @brief List of LIS2DH12 listeners.
     * 
     * This vector contains all the LIS2DH12 listeners that are registered
     * with this component. When a new value is available, the
     * respective callback function of each listener is called.
     */
    std::vector<LIS2DH12Listener *> listeners_{};

    /**
     * @brief Context for communication with the LIS2DH12 sensor.
     * 
     * This structure contains the necessary information for the STMicroelectronics
     * LIS2DH12 driver to communicate with the LIS2DH12 sensor.
     * 
     * The members of this structure are:
     * 
     * - write_reg: A pointer to a function that writes data to a specific register
     *              of the LIS2DH12 sensor.
     * 
     * - read_reg: A pointer to a function that reads data from a specific register
     *             of the LIS2DH12 sensor.
     * 
     * - mdelay: A pointer to a function that causes the calling thread to sleep
     *           for a specified amount of time in miliseconds.
     * 
     * - handle: A pointer to the LIS2DH12Component object.
     */
    stmdev_ctx_t lis2dh12_dev_ctx = {
      .write_reg = platform_write_reg,
      .read_reg = platform_read_reg,
      .mdelay = delay,
      .handle = (void *) this
    };

    /**
     * @brief Reads data from a specified register of the LIS2DH12 sensor.
     * 
     * This function reads data from a specific register of the LIS2DH12 sensor by using
     * the provided handle. It ensures that the handle and buffer pointer are valid
     * before proceeding with the read operation.
     * 
     * @param[in] handle A pointer to the LIS2DH12Component object.
     * @param[in] reg The register address to read from.
     * @param[out] bufp A pointer to the buffer containing the data to read.
     * @param[in] len The length of the data to read.
     * 
     * @return an i2c::ErrorCode
     */
    static int32_t platform_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

    /**
     * @brief Writes data to a specified register of the LIS2DH12 sensor.
     * 
     * This function writes a buffer of data to a specific register of the LIS2DH12 sensor
     * by using the provided handle. It ensures that the handle and buffer pointer are valid
     * before proceeding with the write operation.
     * 
     * @param[in] handle A pointer to the LIS2DH12Component object.
     * @param[in] reg The register address to write to.
     * @param[in] bufp A pointer to the buffer containing the data to write.
     * @param[in] len The length of the data to write.
     * 
     * @return an i2c::ErrorCode indicating the result of the write operation.
     */
    static int32_t platform_write_reg(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
};
;

}  // namespace lis2dh12
}  // namespace esphome
