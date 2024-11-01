#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/hal.h"
#include "lis2dh12_reg.h"  // ST platform-independent driver

namespace esphome {
namespace lis2dh12 {

/// @brief Indicates a successful operation in the ST driver.
static const int32_t ST_SUCCESS = 0;

/**
 * @brief Configuration for enabling click detection on specific axes.
 *
 * This structure allows enabling or disabling click detection
 * on the x, y, and z axes.
 */
struct LIS2DH12ClickAxisConfig {
  /**
   * @brief Click detection on the x axis.
   *
   * - 0: Disable click detection on the x axis
   * 
   * - 1: Enable click detection on the x axis
   */ 
  uint8_t x_en : 1;

  /**
   * @brief Click detection on the y axis.
   *
   * - 0: Disable click detection on the y axis
   * 
   * - 1: Enable click detection on the y axis
   */
  uint8_t y_en : 1;

  /**
   * @brief Click detection on the z axis.
   *
   * - 0: Disable click detection on the z axis
   * 
   * - 1: Enable click detection on the z axis
   */
  uint8_t z_en : 1;
};

/**
 * @brief Compares two `LIS2DH12ClickAxisConfig` objects for equality.
 *
 * @param[in] lhs The left-hand side `LIS2DH12ClickAxisConfig` object.
 * @param[in] rhs The right-hand side `LIS2DH12ClickAxisConfig` object.
 * @return True if both objects have the same x, y, and z axis enable settings; otherwise, false.
 */
inline bool operator==(const LIS2DH12ClickAxisConfig &lhs, const LIS2DH12ClickAxisConfig &rhs) {
  return lhs.x_en == rhs.x_en &&
         lhs.y_en == rhs.y_en &&
         lhs.z_en == rhs.z_en;
}

/**
 * @brief Compares two `LIS2DH12ClickAxisConfig` objects for inequality.
 *
 * @param[in] lhs The left-hand side `LIS2DH12ClickAxisConfig` object.
 * @param[in] rhs The right-hand side `LIS2DH12ClickAxisConfig` object.
 * @return True if the two objects have different x, y, and z axis enable settings; otherwise, false.
 */
inline bool operator!=(const LIS2DH12ClickAxisConfig &lhs, const LIS2DH12ClickAxisConfig &rhs) {
  return !(lhs == rhs);
}

/**
 * @brief Configuration for the timing of click detection.
 *
 * This structure contains fields for controlling the timing
 * of click detection.
 */
struct LIS2DH12TimingConfig {
  /**
   * @brief Time limit for click detection.
   *
   * This field configures the time limit for click detection.
   * See the datasheet for more information. 0-255
   */
  uint8_t time_limit;

  /**
   * @brief Time window for click detection.
   *
   * This field configures the time window for click detection.
   * See the datasheet for more information. 0-255
   */
  uint8_t time_window;

  /**
   * @brief Time latency for click detection.
   *
   * This field configures the time latency for click detection.
   * See the datasheet for more information. 0-255
   */
  uint8_t time_latency;
};

/**
 * @brief Compares two `LIS2DH12TimingConfig` objects for equality.
 *
 * @param[in] lhs The left-hand side `LIS2DH12TimingConfig` object.
 * @param[in] rhs The right-hand side `LIS2DH12TimingConfig` object.
 * @return True if both objects have the same time limit, time window, and time latency settings; otherwise, false.
 */
inline bool operator==(const LIS2DH12TimingConfig &lhs, const LIS2DH12TimingConfig &rhs) {
  return lhs.time_limit == rhs.time_limit &&
         lhs.time_window == rhs.time_window &&
         lhs.time_latency == rhs.time_latency;
}

/**
 * @brief Compares two `LIS2DH12TimingConfig` objects for inequality.
 *
 * @param[in] lhs The left-hand side `LIS2DH12TimingConfig` object.
 * @param[in] rhs The right-hand side `LIS2DH12TimingConfig` object.
 * @return True if the two objects have different time limit, time window, and time latency settings; otherwise, false.
 */
inline bool operator!=(const LIS2DH12TimingConfig &lhs, const LIS2DH12TimingConfig &rhs) {
  return !(lhs == rhs);
}

/**
 * @brief Click detection configuration.
 *
 * This structure contains fields for configuring the click detection
 * feature of the LIS2DH12.
 */
struct LIS2DH12ClickConfig {
  /**
   * @brief Interrupt mode for click detection.
   * 
   * See the ST driver documentation for more information.
   */
  lis2dh12_lir_click_t interrupt_mode;

  /**
   * @brief Threshold for click detection.
   *
   * This field configures the threshold for click detection.
   * See the datasheet for more information. 0-127
   */
  uint8_t threshold : 7;

  /**
   * @brief Double enable for click detection.
   *
   * - 0: Disable double enable for click detection
   * 
   * - 1: Enable double enable for click detection
   */
  uint8_t double_en : 1;

  /**
   * @brief Axis configuration for click detection.
   *
   * @see LIS2DH12ClickAxisConfig
   */
  LIS2DH12ClickAxisConfig axis;

  /**
   * @brief Timing configuration for click detection.
   *
   * @see LIS2DH12TimingConfig
   */
  LIS2DH12TimingConfig timing;

  /**
   * @return true if click detection is enabled for any axis; otherwise, false
   */
  bool is_enabled() const { 
    return  this->axis.x_en == PROPERTY_ENABLE ||
            this->axis.y_en == PROPERTY_ENABLE ||
            this->axis.z_en == PROPERTY_ENABLE;
  }
};

/**
 * @brief Compares two `LIS2DH12ClickConfig` objects for equality.
 *
 * @param[in] lhs The left-hand side `LIS2DH12ClickConfig` object.
 * @param[in] rhs The right-hand side `LIS2DH12ClickConfig` object.
 * @return True if the two objects have the same interrupt mode, threshold,
 *         double enable, axis configuration, and timing configuration; otherwise, false.
 */
inline bool operator==(const LIS2DH12ClickConfig &lhs, const LIS2DH12ClickConfig &rhs) {
  return lhs.interrupt_mode == rhs.interrupt_mode &&
         lhs.threshold == rhs.threshold &&
         lhs.double_en == rhs.double_en &&
         lhs.axis == rhs.axis &&
         lhs.timing == rhs.timing;
}

/**
 * @brief Compares two `LIS2DH12ClickConfig` objects for inequality.
 *
 * @param[in] lhs The left-hand side `LIS2DH12ClickConfig` object.
 * @param[in] rhs The right-hand side `LIS2DH12ClickConfig` object.
 * @return True if the two objects have different interrupt mode, threshold,
 *         double enable, axis configuration, and timing configuration; otherwise, false.
 */
inline bool operator!=(const LIS2DH12ClickConfig &lhs, const LIS2DH12ClickConfig &rhs) {
  return !(lhs == rhs);
}

struct LIS2DH12Config {
  uint8_t block_data_update : 1;

  /**
   * @brief Operation mode.
   *
   * See the ST driver documentation for more information.
   */
  lis2dh12_op_md_t operation_mode;

  /**
   * @brief Full scale range.
   *
   * See the ST driver documentation for more information.
   */
  lis2dh12_fs_t full_scale_range;

  /**
   * @brief Output data rate.
   *
   * See the ST driver documentation for more information.
   */
  lis2dh12_odr_t data_rate;

  /**
   * @brief Temperature sensor enable.
   *
   * See the ST driver documentation for more information.
   */
  lis2dh12_temp_en_t temperature_en;

  /**
   * @brief Click detection configuration.
   *
   * @see LIS2DH12ClickConfig
   */
  LIS2DH12ClickConfig click;
};

/**
 * @brief Equality operator for `LIS2DH12Config`.
 *
 * Compares two `LIS2DH12Config` objects for equality.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 * @return True if the two objects are equal, false otherwise.
 */
inline bool operator==(const LIS2DH12Config &lhs, const LIS2DH12Config &rhs) {
  return lhs.block_data_update == rhs.block_data_update &&
         lhs.operation_mode == rhs.operation_mode &&
         lhs.full_scale_range == rhs.full_scale_range &&
         lhs.data_rate == rhs.data_rate &&
         lhs.temperature_en == rhs.temperature_en &&
         lhs.click == rhs.click;
}

/**
 * @brief Inequality operator for `LIS2DH12Config`.
 *
 * Compares two `LIS2DH12Config` objects for inequality.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 * @return True if the two objects are not equal, false otherwise.
 */
inline bool operator!=(const LIS2DH12Config &lhs, const LIS2DH12Config &rhs) {
  return !(lhs == rhs);
}

/**
 * @brief Structure to hold sensor data.
 *
 * This structure represents the data obtained from a sensor,
 * including x, y, z coordinates and temperature. 
 * If there is an error, data will is NAN.
 */
struct SensorData {
    /**
     * @brief X acceleration in m/s^2.
     */
    float x; 

    /**
     * @brief X acceleration in m/s^2.
     */
    float y; 

    /**
     * @brief X acceleration in m/s^2.
     */
    float z; 

    /**
     * @brief Temperature in degrees Celsius.
     */
    float temperature; 

    /**
     * @brief Converts the sensor data to a string representation.
     *
     * This function formats the sensor data (x, y, z, temperature)
     * into a string with three decimal points.
     *
     * @return A string representation of the sensor data.
     */
    std::string to_string() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "x: " << x << ", "
            << "y: " << y << ", "
            << "z: " << z << ", "
            << "t: " << temperature;
        return oss.str();
    }
};

/**
 * @brief Default LIS2DH12 configuration.
 */
static const LIS2DH12Config LIS2DH12_Default_Config = {
  .block_data_update = PROPERTY_ENABLE, // Updates all sensor data registers at once
  .operation_mode = LIS2DH12_HR_12bit, // High resolution mode with a resolution of 12 bits
  .full_scale_range = LIS2DH12_2g, // Gives maximum range of +/- 2g for best precision
  .data_rate = LIS2DH12_ODR_400Hz, // 2.5 ms refresh rate
  .temperature_en = LIS2DH12_TEMP_DISABLE, // Disabled if not needed
  .click = {
      .interrupt_mode = LIS2DH12_TAP_LATCHED, // Keeps interrupt on until click source is read
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

/**
 * @enum OperationMode
 *
 * @brief Enumeration for operation mode settings of the LIS2DH12 component.
 */
enum OperationMode {
  LOW_POWER                       = lis2dh12_op_md_t::LIS2DH12_LP_8bit, ///< Low power mode with a resolution of 8 bits.
  NORMAL                          = lis2dh12_op_md_t::LIS2DH12_NM_10bit, ///< Normal mode with a resolution of 10 bits.
  HIGH_RESOLUTION                 = lis2dh12_op_md_t::LIS2DH12_HR_12bit ///< High resolution mode with a resolution of 12 bits.
};

/**
 * @enum DataRate
 *
 * @brief Enumeration for data rate settings of the LIS2DH12 component.
 *
 * @details
 * This enumeration defines the possible output data rates for the LIS2DH12
 * accelerometer. Each value corresponds to a specific data rate setting 
 * in Hz. It allows the user to configure how frequently the sensor 
 * updates its data.
 */
enum DataRate {
  POWER_DOWN                      = lis2dh12_odr_t::LIS2DH12_POWER_DOWN, ///< Power-down mode.
  ODR_1HZ                         = lis2dh12_odr_t::LIS2DH12_ODR_1Hz, ///< 1 Hz data rate.
  ODR_10HZ                        = lis2dh12_odr_t::LIS2DH12_ODR_10Hz, ///< 10 Hz data rate.
  ODR_25HZ                        = lis2dh12_odr_t::LIS2DH12_ODR_25Hz, ///< 25 Hz data rate.
  ODR_50HZ                        = lis2dh12_odr_t::LIS2DH12_ODR_50Hz, ///< 50 Hz data rate.
  ODR_100HZ                       = lis2dh12_odr_t::LIS2DH12_ODR_100Hz, ///< 100 Hz data rate.
  ODR_200HZ                       = lis2dh12_odr_t::LIS2DH12_ODR_200Hz, ///< 200 Hz data rate.
  ODR_400HZ                       = lis2dh12_odr_t::LIS2DH12_ODR_400Hz, ///< 400 Hz data rate.
  ODR_1KHZ620_LP                  = lis2dh12_odr_t::LIS2DH12_ODR_1kHz620_LP, ///< 1620 Hz data rate only in low power mode.
  ODR_5KHZ376_LP_1KHZ344_NM_HP    = lis2dh12_odr_t::LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP ///< 5376 Hz data rate only in low power mode and 1344 Hz data rate in normal mode and high resolution mode.
};

/**
 * @enum FullScaleRange
 *
 * @brief LIS2DH12 component full scale range settings.
 *
 * @details
 * These codes are used to represent the full scale range of the device in the
 * LIS2DH12Component.
 */
enum FullScaleRange {
  FS_2G   = lis2dh12_fs_t::LIS2DH12_2g, ///< Gives maximum range of +/- 2g with highest precision
  FS_4G   = lis2dh12_fs_t::LIS2DH12_4g, ///< Gives maximum range of +/- 4g with good precision
  FS_8G   = lis2dh12_fs_t::LIS2DH12_8g, ///< Gives maximum range of +/- 8g with medium precision
  FS_16G  = lis2dh12_fs_t::LIS2DH12_16g ///< Gives maximum range of +/- 16g with lowest precision
};

/**
 * @enum StateCode
 *
 * @brief LIS2DH12 component state codes.
 *
 * @details
 * These codes are used to represent the state of the device in the
 * LIS2DH12Component.
 */
enum StateCode : uint8_t {
  READY_OK = 0, ///< The device is ready and working.
  SETUP_STAGE_0, ///< The component is identifying LIS2DH12.
  SETUP_STAGE_1, ///< The component is configuring LIS2DH12.
  SEARCHING_FOR_DEVICE, ///< The component is searching for the device.
  CONFIGURING_DEVICE, ///< The component is configuring the device.
  WAITING_FOR_ACCEL_DATA, ///< The component is waiting for new acceleration data.
  WAITING_FOR_TEMP_DATA, ///< The component is waiting for new temperature data.
  GOT_REQUESTED_DATA, ///< New data is available.
  BUSY, ///< The component is busy.
  ERROR, ///< An error has occurred.
  ERROR_TIMEOUT, ///< A timeout has occurred.
  ERROR_NOT_FOUND, ///< The device was not found.
  ERROR_NOT_RESPONDING, ///< The device is not responding.
  ERROR_NOT_CONFIGURED ///< The device is not configured.
};

/**
 * @brief Interface for LIS2DH12 event listeners.
 */
class LIS2DH12Listener {
  public:
    /**
     * @brief Called when the device is ready.
     */
    virtual void on_ready() { this->lis2dh12_ready_ = true; }

    /**
     * @brief Called when the device has failed.
     */
    virtual void on_failure() { this->lis2dh12_ready_ = false; }

    /**
     * @brief Check if the listener is currently ready.
     *
     * This method returns the state of the listener, indicating whether
     * it is ready to receive and process data.
     *
     * @return True if the listener is ready, false otherwise.
     */
    bool is_listening() { return this->lis2dh12_ready_; }

    /**
     * @brief Called when the configuration has changed.
     *
     * @param[in] newConfig The new configuration.
     * @param[in] oldConfig The old configuration.
     */
    virtual void on_config_change(LIS2DH12Config newConfig, LIS2DH12Config oldConfig) {};
    
    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~LIS2DH12Listener() = default;  

  protected:
    /**
     * @brief Flag indicating whether the listener is ready.
     *
     * This flag is set to true when the listener is ready and false when it is not.
     * The `on_ready` and `on_failure` methods change state of this flag.
     */
    bool lis2dh12_ready_ = false;
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
     * @brief Returns true if the sensor data needs to be updated.
     *
     * @return True if listener is listening and the sensor data needs to be updated.
     */
    virtual bool needs_update() { return this->is_listening() && this->needs_update_; }

    /**
     * @brief Requests an update.
     */
    virtual void request_update() { this->needs_update_ = true; }

    /**
     * @brief Called when the sensor data has been updated.
     *
     * @param[in] data The updated sensor data.
     */
    virtual void on_sensor_update(SensorData data) { this->needs_update_ = false; }

    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~LIS2DH12SensorListener() = default;  

  protected:
    /**
     * @brief Flag indicating whether the sensor data needs to be updated.
     *
     * This flag is set to true when an update is requested, and set to false
     * when the sensor data has been updated.
     */
    bool needs_update_ = false;
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

    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~LIS2DH12ClickListener() = default;  
};

/**
 * @brief `LIS2DH12Component` class responsible for interfacing with the LIS2DH12 sensor.
 * 
 * This class provides methods for configuring and communicating with the LIS2DH12 sensor.
 * It allows setting operation modes, data rates, full-scale ranges, and enabling the internal
 * temperature sensor. It also manages a list of listeners that can receive various events
 * and data updates from the sensor.
 */
class LIS2DH12Component : public Component, public i2c::I2CDevice {
  public:
    /**
     * @brief Marks the component as failed.
     *
     * This method will call the base class method to mark the component as failed.
     * Additionally, it will notify all general listeners of the failure.
     */
    void mark_failed() override;
    void setup() override;
    void loop() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::HARDWARE; };

    /**
     * @brief Sets the operation mode of the LIS2DH12 component.
     * 
     * @param[in] mode The new operation mode.
     */
    void set_operation_mode(OperationMode mode) { new_config_.operation_mode = static_cast<lis2dh12_op_md_t>(mode); }

    /**
     * @brief Sets the full scale range of the LIS2DH12 component.
     * 
     * @param[in] full_scale_range The new full scale range.
     */
    void set_full_scale_range(FullScaleRange full_scale_range) { new_config_.full_scale_range = static_cast<lis2dh12_fs_t>(full_scale_range); }

    /**
     * @brief Sets the data rate of the LIS2DH12 component.
     * 
     * @param[in] rate The new data rate.
     */
    void set_data_rate(DataRate rate) { new_config_.data_rate = static_cast<lis2dh12_odr_t>(rate); }
    
    /**
     * @brief Enables the internal temperature sensor of the LIS2DH12 component.
     */
    void enable_internal_temperature_sensor() { new_config_.temperature_en = LIS2DH12_TEMP_ENABLE;}

    /**
     * @brief Registers a listener to receive events from the LIS2DH12 component.
     * 
     * @param[in] listener The listener to register.
     */
    void register_listener(LIS2DH12Listener *listener) { general_listeners_.push_back(listener); }

    /**
     * @brief Registers a listener to receive sensor data from the LIS2DH12 component.
     * 
     * @param[in] listener The listener to register.
     */
    void register_sensor_listener(LIS2DH12SensorListener *listener) { sensor_listeners_.push_back(listener); 
                                                                      register_listener(listener); }

    /**
     * @brief Registers a listener to receive click events from the LIS2DH12 component.
     * 
     * @param[in] listener The listener to register.
     */
    void register_click_listener(LIS2DH12ClickListener *listener) { click_listeners_.push_back(listener);
                                                                    register_listener(listener); }

  protected:

    SensorData get_sensor_data(bool includeTemperature = false);

    /**
     * @brief Get the temperature in Celsius from the LIS2DH12 sensor.
     * 
     * This function reads the raw temperature data from the sensor,
     * determines the sensor's operating mode, and converts the raw data to Celsius.
     * 
     * @return The temperature in Celsius if successful, NAN otherwise.
     */
    float get_temperature_celsius();

    bool apply_device_configuration(LIS2DH12Config config, bool force = false);

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
     * @brief Notify all registered click listeners about a click event.
     *
     * This function notifies all registered click listeners about a click event. It
     * iterates over the list of registered click listeners and calls their
     * `on_click` method with the provided click source.
     *
     * @param[in] clickSource The source of the click.
     * @return The number of registered click listeners.
     */
    size_t click_listeners_on_click(lis2dh12_click_src_t clickSrc);

    /**
     * @brief Notify all sensor listeners of a sensor data update.
     *
     * This function iterates over all registered sensor listeners and calls their
     * `on_sensor_update` method with the provided sensor data. It checks if each listener
     * needs an update before calling the method. If a listener is null, it logs an error.
     *
     * @param[in] data The updated sensor data to be sent to the listeners.
     * @return The number of listeners that were notified.
     */
    size_t sensor_listeners_on_sensor_update(SensorData data);

    /**
     * @brief Notify all listeners of a configuration change.
     *
     * This function iterates over all registered general listeners and calls their
     * `on_config_change` method, updating them with the new and old configuration.
     * If the configuration hasn't changed and force is not true, the function returns early.
     *
     * @param[in] newConfig The new configuration settings.
     * @param[in] oldConfig The previous configuration settings.
     * @param[in] force If true, notify listeners even if the configuration hasn't changed.
     * @return The number of listeners that were notified.
     */
    size_t general_listeners_on_config_change(LIS2DH12Config newConfig, LIS2DH12Config oldConfig, bool force = false);

    /**
     * @brief Notify all listeners that the LIS2DH12 device is ready.
     *
     * This function notifies all listeners registered with the LIS2DH12 component
     * that the device is ready to be used.  It iterates over all listeners and
     * calls their `on_ready` method.
     *
     * @return The number of listeners notified.
     */
    size_t general_listeners_on_ready();

    /**
     * @brief Notify all listeners that the LIS2DH12 component failed.
     *
     * This function notifies all listeners registered with the LIS2DH12 component
     * that the component failed.  It iterates over all listeners and
     * calls their `on_failed` method.
     *
     * @return The number of listeners notified.
     */
    size_t general_listeners_on_failure();

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
     * @brief The current configuration for the LIS2DH12 component.
     *
     * @see LIS2DH12Config
     */
    LIS2DH12Config current_config_ = LIS2DH12_Default_Config;

    /**
     * @brief The new configuration for the LIS2DH12 component.
     *
     * @see LIS2DH12Config
     */
    LIS2DH12Config new_config_= LIS2DH12_Default_Config;

    /**
     * @brief Has the setup procedure been done?
     *
     * @see setup()
     */
    bool setup_done_ = false;

    /**
     * @brief The current state of the LIS2DH12 component.
     *
     * @see StateCode
     */
    StateCode state_ = StateCode::ERROR_NOT_FOUND;

    /**
     * @brief Context for communication with the LIS2DH12 sensor.
     * 
     * This structure contains the necessary information for the STMicroelectronics
     * LIS2DH12 driver to communicate with the LIS2DH12 sensor.
     * 
     * The members of this structure are:
     * 
     * - `write_reg`: A pointer to a function that writes data to a specific register
     *              of the LIS2DH12 sensor.
     * 
     * - `read_reg`: A pointer to a function that reads data from a specific register
     *             of the LIS2DH12 sensor.
     * 
     * - `mdelay`: A pointer to a function that causes the calling thread to sleep
     *           for a specified amount of time in miliseconds.
     * 
     * - `handle`: A pointer to the LIS2DH12Component object.
     */
    stmdev_ctx_t lis2dh12_dev_ctx_ = {
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
