#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/hal.h"
#include "lis2dh12_reg.h"  // ST platform-independent driver

namespace esphome {
namespace lis2dh12 {

// ST driver return code
static const int32_t ST_SUCCESS = 0;

struct LIS2DH12ClickAxisConfig {
  uint8_t x_en : 1;
  uint8_t y_en : 1;
  uint8_t z_en : 1;
};

inline bool operator==(const LIS2DH12ClickAxisConfig &lhs, const LIS2DH12ClickAxisConfig &rhs) {
  return lhs.x_en == rhs.x_en &&
         lhs.y_en == rhs.y_en &&
         lhs.z_en == rhs.z_en;
}

inline bool operator!=(const LIS2DH12ClickAxisConfig &lhs, const LIS2DH12ClickAxisConfig &rhs) {
  return !(lhs == rhs);
}

struct LIS2DH12TimingConfig {
  uint8_t time_limit;
  uint8_t time_window;
  uint8_t time_latency;
};

inline bool operator==(const LIS2DH12TimingConfig &lhs, const LIS2DH12TimingConfig &rhs) {
  return lhs.time_limit == rhs.time_limit &&
         lhs.time_window == rhs.time_window &&
         lhs.time_latency == rhs.time_latency;
}

inline bool operator!=(const LIS2DH12TimingConfig &lhs, const LIS2DH12TimingConfig &rhs) {
  return !(lhs == rhs);
}

struct LIS2DH12ClickConfig {
  lis2dh12_lir_click_t interrupt_mode;
  uint8_t threshold;
  uint8_t double_en : 1;
  LIS2DH12ClickAxisConfig axis;
  LIS2DH12TimingConfig timing;
  bool is_enabled() const { 
    return  this->axis.x_en == PROPERTY_ENABLE ||
            this->axis.y_en == PROPERTY_ENABLE ||
            this->axis.z_en == PROPERTY_ENABLE;
  }
};

inline bool operator==(const LIS2DH12ClickConfig &lhs, const LIS2DH12ClickConfig &rhs) {
  return lhs.interrupt_mode == rhs.interrupt_mode &&
         lhs.threshold == rhs.threshold &&
         lhs.double_en == rhs.double_en &&
         lhs.axis == rhs.axis &&
         lhs.timing == rhs.timing;
}

inline bool operator!=(const LIS2DH12ClickConfig &lhs, const LIS2DH12ClickConfig &rhs) {
  return !(lhs == rhs);
}

struct LIS2DH12Config {
  uint8_t block_data_update : 1;
  lis2dh12_op_md_t operation_mode;
  lis2dh12_fs_t full_scale_range;
  lis2dh12_odr_t data_rate;
  lis2dh12_temp_en_t temperature_en;
  LIS2DH12ClickConfig click;
};

inline bool operator==(const LIS2DH12Config &lhs, const LIS2DH12Config &rhs) {
  return lhs.block_data_update == rhs.block_data_update &&
         lhs.operation_mode == rhs.operation_mode &&
         lhs.full_scale_range == rhs.full_scale_range &&
         lhs.data_rate == rhs.data_rate &&
         lhs.temperature_en == rhs.temperature_en &&
         lhs.click == rhs.click;
}

inline bool operator!=(const LIS2DH12Config &lhs, const LIS2DH12Config &rhs) {
  return !(lhs == rhs);
}

static const LIS2DH12Config LIS2DH12_Default_Config = {
  .block_data_update = PROPERTY_ENABLE,
  .operation_mode = LIS2DH12_HR_12bit,
  .full_scale_range = LIS2DH12_2g,
  .data_rate = LIS2DH12_ODR_400Hz,
  .temperature_en = LIS2DH12_TEMP_DISABLE,
  .click = {
      .interrupt_mode = LIS2DH12_TAP_LATCHED,
      .threshold = 0,
      .double_en = PROPERTY_DISABLE,
      .axis = {
          .x_en = PROPERTY_DISABLE,
          .y_en = PROPERTY_DISABLE,
          .z_en = PROPERTY_DISABLE
      },
      .timing = {
          .time_limit = 0,
          .time_window = 0,
          .time_latency = 0
      }
  }
};

enum OperationMode {
  LOW_POWER                       = lis2dh12_op_md_t::LIS2DH12_LP_8bit,
  NORMAL                          = lis2dh12_op_md_t::LIS2DH12_NM_10bit,
  HIGH_RESOLUTION                 = lis2dh12_op_md_t::LIS2DH12_HR_12bit
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

/**
 * @brief Interface for LIS2DH12 event listeners.
 */
class LIS2DH12Listener {
  public:
    /**
     * @brief Called when the device is ready.
     */
    virtual void on_ready() {};

    /**
     * @brief Called when the device has failed.
     */
    virtual void on_failure() {};

    /**
     * @brief Called when the configuration has changed.
     * 
     * @param config The new configuration.
     */
    virtual void on_config_change(LIS2DH12Config config) {};
    
    // /**
    //  * @brief Virtual destructor for proper cleanup
    //  */
    // virtual ~LIS2DH12Listener() = default;  
};

/**
 * @brief Interface for LIS2DH12 sensor listeners.
 *
 * This interface is used by the LIS2DH12 component to send sensor data to
 * registered listeners.
 */
class LIS2DH12SensorListener : public LIS2DH12Listener {
  public:
    /**
     * @brief Queries whether the listener needs an update.
     *
     * @return true if the listener needs an update, false otherwise.
     */
    virtual bool needs_update() = 0;

    /**
     * @brief Called when the sensor data has been updated.
     *
     * @param value The updated sensor value.
     */
    virtual void on_sensor_update(float value) {};

    // /**
    //  * @brief Virtual destructor for proper cleanup
    //  */
    // virtual ~LIS2DH12SensorListener() = default;  
};

/**
 * @brief Interface for LIS2DH12 click event listeners.
 *
 * This interface is used to receive click events from the LIS2DH12 component.
 */
class LIS2DH12ClickListener : public LIS2DH12Listener {
  public:
    /**
     * @brief Called when a click is detected.
     *
     * @param clickSrc The source of the click.
     */
    virtual void on_click(lis2dh12_click_src_t clickSrc) {};

    // /**
    //  * @brief Virtual destructor for proper cleanup
    //  */
    // virtual ~LIS2DH12ClickListener() = default;  
};

class LIS2DH12Component : public Component, public i2c::I2CDevice {
  public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::HARDWARE; };

    /**
     * @brief Sets the operation mode of the LIS2DH12 component.
     * 
     * @param mode The new operation mode.
     */
    void set_operation_mode(OperationMode mode) { new_config_.operation_mode = static_cast<lis2dh12_op_md_t>(mode); }

    /**
     * @brief Sets the full scale range of the LIS2DH12 component.
     * 
     * @param full_scale_range The new full scale range.
     */
    void set_full_scale_range(FullScaleRange full_scale_range) { new_config_.full_scale_range = static_cast<lis2dh12_fs_t>(full_scale_range); }

    /**
     * @brief Sets the data rate of the LIS2DH12 component.
     * 
     * @param rate The new data rate.
     */
    void set_data_rate(DataRate rate) { new_config_.data_rate = static_cast<lis2dh12_odr_t>(rate); }
    
    /**
     * @brief Enables the internal temperature sensor of the LIS2DH12 component.
     */
    void enable_internal_temperature_sensor() { new_config_.temperature_en = LIS2DH12_TEMP_ENABLE;}

    /**
     * @brief Registers a listener to receive events from the LIS2DH12 component.
     * 
     * @param listener The listener to register.
     */
    void register_listener(LIS2DH12Listener *listener) { general_listeners_.push_back(listener); }

    /**
     * @brief Registers a listener to receive sensor data from the LIS2DH12 component.
     * 
     * @param listener The listener to register.
     */
    void register_sensor_listener(LIS2DH12SensorListener *listener) { sensor_listeners_.push_back(listener); }

    /**
     * @brief Registers a listener to receive click events from the LIS2DH12 component.
     * 
     * @param listener The listener to register.
     */
    void register_click_listener(LIS2DH12ClickListener *listener) { click_listeners_.push_back(listener); }

  protected:
    StateCode state_ = StateCode::ERROR_NOT_FOUND;
    
    LIS2DH12Config current_config_ = LIS2DH12_Default_Config;
    LIS2DH12Config new_config_= LIS2DH12_Default_Config;

    void apply_device_configuration(LIS2DH12Config config, bool force = false);

    /**
     * @brief Setup the LIS2DH12 component
     * 
     * This function is called from the setup() method of the component. It
     * performs the necessary steps to setup the LIS2DH12 component.
     * 
     * @param[in] stage The current stage of the setup process.
     */
    void internal_setup(uint8_t stage);

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
     * @return True if the reset is successful within the timeout, false otherwise.
     */
    [[deprecated("Manually set registers to default value instead.")]]
    bool reset_device(uint8_t timeout_ms = 10);

    /**
     * @brief List of LIS2DH12 listeners.
     *
     * This vector contains all the LIS2DH12 listeners that are registered
     * with this component. When a new value is available, the
     * respective callback function of each listener is called.
     *
     * @see LIS2DH12Listener
     */
    std::vector<LIS2DH12Listener *> general_listeners_;

    /**
     * @brief List of LIS2DH12 sensor listeners.
     *
     * This vector contains all the LIS2DH12 sensor listeners that are registered
     * with this component. When new sensor data is available, the
     * respective callback function of each sensor listener is called.
     *
     * @see LIS2DH12SensorListener
     */
    std::vector<LIS2DH12SensorListener *> sensor_listeners_;

    /**
     * @brief List of LIS2DH12 click listeners.
     *
     * This vector contains all the LIS2DH12 click listeners that are registered
     * with this component. When a click event is detected, the
     * respective callback function of each click listener is called.
     *
     * @see LIS2DH12ClickListener
     */
    std::vector<LIS2DH12ClickListener *> click_listeners_;

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


}  // namespace lis2dh12
}  // namespace esphome
