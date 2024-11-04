#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "lis2dh12.h"
#include "lis2dh12_reg.h"  // ST platform-independent driver

namespace esphome {
namespace lis2dh12 {

/**
 * @brief The tag used for logging
 */
static const char *const TAG = "lis2dh12";

/**
 * @brief Delay after applying configuration.
 * 
 * The chip needs some time to process the changes.
 * According to the ST AN5005, 7ms is recommended.
 */
static const uint8_t TURN_ON_DELAY_MS = 7;

#define LIS2DH12_CONFIGURATION_ERROR_CHECK(conf_current, conf_new, force, conf_log, func, error, info) \
  if ( this->state_ == CONFIGURING_DEVICE ) { \
    if ( (conf_current != conf_new) || force) { \
      ESP_LOGCONFIG(TAG, conf_log); \
      if (func != ST_SUCCESS) { \
        ESP_LOGE(TAG, error); \
        this->state_ = ERROR_NOT_CONFIGURED; \
      } \
      conf_current = conf_new; \
    } else { \
      ESP_LOGV(TAG, info); \
    } \
  }


void LIS2DH12Component::mark_failed() {
  // Call the base class method to mark the component as failed
  Component::mark_failed();

  // Notify all general listeners of the failure
  this->general_listeners_on_failure();
}

void LIS2DH12Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LIS2DH12:");
  LOG_I2C_DEVICE(this);
  
  switch (this->state_) {
    case StateCode::READY_OK:
      ESP_LOGCONFIG(TAG, "  State: Running normally");
      break;
    case StateCode::SEARCHING_FOR_DEVICE:
      ESP_LOGCONFIG(TAG, "  State: Searching for device");
      break;
    case StateCode::CONFIGURING_DEVICE:
      ESP_LOGCONFIG(TAG, "  State: Configuring device");
      break;
    case StateCode::WAITING_FOR_ACCEL_DATA:
      ESP_LOGCONFIG(TAG, "  State: Waiting for acceleration data");
      break;
    case StateCode::WAITING_FOR_TEMP_DATA:
      ESP_LOGCONFIG(TAG, "  State: Waiting for temperature data");
      break;
    case StateCode::GOT_REQUESTED_DATA:
      ESP_LOGCONFIG(TAG, "  State: Got requested sensor data");
      break;
    case StateCode::BUSY:
      ESP_LOGCONFIG(TAG, "  State: Busy");
      break;
    case StateCode::ERROR:
      ESP_LOGE(TAG, "  State: Unspecified error");
      break;
    case StateCode::ERROR_TIMEOUT:
      ESP_LOGE(TAG, "  State: LIS2DH12 timed out");
      break;
    case StateCode::ERROR_NOT_FOUND:
      ESP_LOGE(TAG, "  State: LIS2DH12 not found");
      break;
    case StateCode::ERROR_NOT_RESPONDING:
      ESP_LOGE(TAG, "  State: LIS2DH12 not responding");
      break;
    case StateCode::ERROR_NOT_CONFIGURED:
      ESP_LOGE(TAG, "  State: LIS2DH12 configuration failed");
      break;
    default:
      ESP_LOGW(TAG, "  State: Unknown");
      break;
  }

  if(this->general_listeners_.size() > 0) {
    size_t totalListenersCount = this->general_listeners_.size();
    size_t sensorListenersCount = this->sensor_listeners_.size();
    size_t clickListenersCount = this->click_listeners_.size();
    size_t generalListenersCount = totalListenersCount - sensorListenersCount - clickListenersCount;
    ESP_LOGCONFIG(TAG, "  Listeners:");
    if (generalListenersCount > 0)
      ESP_LOGCONFIG(TAG, "    General: %zu", generalListenersCount);
    if (sensorListenersCount > 0)
      ESP_LOGCONFIG(TAG, "    Sensor:  %zu", sensorListenersCount);
    if (clickListenersCount > 0)
      ESP_LOGCONFIG(TAG, "    Click:   %zu", clickListenersCount);
    if (generalListenersCount != totalListenersCount && generalListenersCount > 0)
      ESP_LOGCONFIG(TAG, "    Total:   %zu", totalListenersCount);
  }

  ESP_LOGCONFIG(TAG, "  Block Data Update: %s", ONOFF(this->current_config_.block_data_update == PROPERTY_ENABLE));

  if (this->current_config_.data_rate == LIS2DH12_POWER_DOWN) {
    ESP_LOGCONFIG(TAG, "  Operation mode: Power down");
  } else {
    switch (this->current_config_.operation_mode) {
      case LIS2DH12_LP_8bit:
        ESP_LOGCONFIG(TAG, "  Operation mode: Low power (8-bit)");
        break;
      case LIS2DH12_NM_10bit:
        ESP_LOGCONFIG(TAG, "  Operation mode: Normal (10-bit)");
        break;
      case LIS2DH12_HR_12bit:
        ESP_LOGCONFIG(TAG, "  Operation mode: High resolution (12-bit)");
        break;
      default:
        ESP_LOGW(TAG, "  Operation mode: Unknown");
        break;
    }
  }

  switch (this->current_config_.full_scale_range) {
    case LIS2DH12_2g:
      ESP_LOGCONFIG(TAG, "  Full scale range: +/- 2g");
      break;
    case LIS2DH12_4g:
      ESP_LOGCONFIG(TAG, "  Full scale range: +/- 4g");
      break;
    case LIS2DH12_8g:
      ESP_LOGCONFIG(TAG, "  Full scale range: +/- 8g");
      break;
    case LIS2DH12_16g:
      ESP_LOGCONFIG(TAG, "  Full scale range: +/- 16g");
      break;
    default:
      ESP_LOGW(TAG, "  Full scale range: Unknown");
      break;
  }

  switch (this->current_config_.data_rate) {
    case LIS2DH12_POWER_DOWN:
      ESP_LOGCONFIG(TAG, "  Data rate: Power down");
      break;
    case LIS2DH12_ODR_1Hz:
      ESP_LOGCONFIG(TAG, "  Data rate: 1 Hz");
      break;
    case LIS2DH12_ODR_10Hz:
      ESP_LOGCONFIG(TAG, "  Data rate: 10 Hz");
      break;
    case LIS2DH12_ODR_25Hz:
      ESP_LOGCONFIG(TAG, "  Data rate: 25 Hz");
      break;
    case LIS2DH12_ODR_50Hz:
      ESP_LOGCONFIG(TAG, "  Data rate: 50 Hz");
      break;
    case LIS2DH12_ODR_100Hz:
      ESP_LOGCONFIG(TAG, "  Data rate: 100 Hz");
      break;
    case LIS2DH12_ODR_200Hz:
      ESP_LOGCONFIG(TAG, "  Data rate: 200 Hz");
      break;
    case LIS2DH12_ODR_400Hz:
      ESP_LOGCONFIG(TAG, "  Data rate: 400 Hz");
      break;
    case LIS2DH12_ODR_1kHz620_LP:
      if(this->current_config_.operation_mode == LIS2DH12_LP_8bit) {
        ESP_LOGCONFIG(TAG, "  Data rate: 1620 Hz");
      } else {
        ESP_LOGW(TAG, "  Data rate: 1620 Hz is only supported in low power mode");
      }
      break;
    case LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP:
      if(this->current_config_.operation_mode == LIS2DH12_LP_8bit) {
        ESP_LOGCONFIG(TAG, "  Data rate: 5376 Hz");
      } else {
        ESP_LOGCONFIG(TAG, "  Data rate: 1344 Hz");
      }
      break;
    default:
      ESP_LOGW(TAG, "  Data rate: Unknown");
      break;
  }

  ESP_LOGCONFIG(TAG, "  Internal temperature sensor: %s", ONOFF(this->current_config_.temperature_en == LIS2DH12_TEMP_ENABLE));

  bool clickEnabled = this->current_config_.click.is_enabled();
  ESP_LOGCONFIG(TAG, "  Click detection: %s", ONOFF(clickEnabled));
  if(clickEnabled || click_listeners_.size() > 0) {
    ESP_LOGCONFIG(TAG, "    Double click: %s", ONOFF(current_config_.click.double_en == PROPERTY_ENABLE));
    ESP_LOGCONFIG(TAG, "    Interrupt mode: %s", (current_config_.click.interrupt_mode == LIS2DH12_TAP_LATCHED) ? "Active until read" : "Active for duration of latency window");
    ESP_LOGCONFIG(TAG, "    Threshold: %d (0x%02X)", current_config_.click.threshold, current_config_.click.threshold);
    ESP_LOGCONFIG(TAG, "    Axis:");
    ESP_LOGCONFIG(TAG, "      X: %s", ONOFF(current_config_.click.axis.x_en == PROPERTY_ENABLE));
    ESP_LOGCONFIG(TAG, "      Y: %s", ONOFF(current_config_.click.axis.y_en == PROPERTY_ENABLE)); 
    ESP_LOGCONFIG(TAG, "      Z: %s", ONOFF(current_config_.click.axis.z_en == PROPERTY_ENABLE));
    ESP_LOGCONFIG(TAG, "    Timing:");
    ESP_LOGCONFIG(TAG, "      Limit:   %d (0x%02X)", current_config_.click.timing.time_limit, current_config_.click.timing.time_limit);
    ESP_LOGCONFIG(TAG, "      Window:  %d (0x%02X)", current_config_.click.timing.time_window, current_config_.click.timing.time_window);
    ESP_LOGCONFIG(TAG, "      Latency: %d (0x%02X)", current_config_.click.timing.time_latency, current_config_.click.timing.time_latency);
  }
}


void LIS2DH12Component::setup() {
  this->state_ = SETUP_STAGE_0;
  this->setup_done_ = false;
  ESP_LOGCONFIG(TAG, "Setting up LIS2DH12...");

  ESP_LOGCONFIG(TAG, "  Identifying device...");
  if (!this->is_device_present()) {
    ESP_LOGE(TAG, "Device not found, state = %d", this->state_);
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "  Configuring device...");
  this->state_ = SETUP_STAGE_1;
  this->status_set_warning("configuring");
  if(!(this->apply_device_configuration(this->new_config_, true))) {
    ESP_LOGE(TAG, "Failed to configure device, state = %d", this->state_);
    this->mark_failed();
    return;
  }

  // Mark as failed if state is unexpected
  if (this->state_ != CONFIGURING_DEVICE && this->state_ != SETUP_STAGE_1) {
    ESP_LOGE(TAG, "Unexpected state: %d", this->state_);
    this->mark_failed();
    return;
  }
}

void LIS2DH12Component::loop() {
  // Check if last step of setup is complete
  if (!this->setup_done_ && this->state_ == SETUP_STAGE_1) {
    // Mark setup as done and set state to READY_OK
    this->setup_done_ = true;
    this->state_ = READY_OK;
    this->status_clear_warning();
    ESP_LOGCONFIG(TAG, "Setup complete.");
    this->general_listeners_on_ready();
  }

  // If setup is not done, return
  if (!this->setup_done_) {
    return;
  }

  // Skip loop if configuration is in progress
  if (this->state_ == CONFIGURING_DEVICE) {
    ESP_LOGD(TAG, "Configuration in progress, skipping loop...");
    return;
  }

  // Mark as failed if state is unexpected
  if (this->state_ != READY_OK && 
      this->state_ != WAITING_FOR_ACCEL_DATA && 
      this->state_ != WAITING_FOR_TEMP_DATA &&
      this->state_ != GOT_REQUESTED_DATA) {
    ESP_LOGE(TAG, "Unexpected state: %d", this->state_);
    this->mark_failed();
    return;
  }
  // Normal operation

  // Check for click interrupt if needed
  if (this->click_listeners_.size() > 0 && this->current_config_.click.is_enabled()) {
    lis2dh12_click_src_t click_source;
    if(lis2dh12_tap_source_get(&this->lis2dh12_dev_ctx_, &click_source) != ST_SUCCESS) {
      ESP_LOGE(TAG, "Failed to get click source");
      this->state_ = ERROR_NOT_RESPONDING;
      this->mark_failed();
      return;
    }

    // If click interrupt flag is set, notify listeners
    if (click_source.ia == PROPERTY_ENABLE) {
      this->click_listeners_on_click(click_source);
    }
  }

  // Set to true if at least one sensor listener needs temperature
  bool needsTemperature = false;
  // Listener data reading
  switch(this->state_) {
    case READY_OK:
      // Set to true if any sensor listener needs an update
      bool readSensorData = false;
      // TODO: Compare speed of `for (auto *x : std::vector<*T>)` vs `std::any_of()`
      // TODO: Compare speed of one combined for loop vs two separate for loops
      size_t totalCount = sensor_listeners_.size();
      size_t i = 0;
      // Find first sensor listener that needs an update, needsTemperature will be set to true if needed
      for (i = 0; !readSensorData && (i < totalCount); i++) {
        if (sensor_listeners_[i] != nullptr) {
          readSensorData |= sensor_listeners_[i]->needs_update(&needsTemperature);
        }
      }
      // Check if other listeners need temperature only if none of the sensor listeners needed it yet
      for(i++; !needsTemperature && (i < totalCount); i++) {
        if (sensor_listeners_[i] != nullptr) {
          sensor_listeners_[i]->needs_update(&needsTemperature);
        }
      }
      // Break if no sensor listeners need data, return and repeat listener check in next loop
      if (!readSensorData) {
        break;
      }
      // If any sensor listener needs an update, wait for new data
      if (needsTemperature && (this->current_config_.temperature_en != LIS2DH12_TEMP_ENABLE)) {
        ESP_LOGW(TAG, "Sensor listeners requested temperature data, but internal temperature sensor is not enabled.");
        needsTemperature = false;
      }     
      this->state_ = WAITING_FOR_ACCEL_DATA;
    case WAITING_FOR_ACCEL_DATA:
      // If waiting for accel data, check if accel data is ready
      uint8_t accelDataReady;
      if(lis2dh12_xl_data_ready_get(&this->lis2dh12_dev_ctx_, &accelDataReady) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed to read status register");
        this->state_ = ERROR_NOT_RESPONDING;
        this->mark_failed();
        return;
      }
      // If no data ready, return and repeat in next loop
      if (accelDataReady != PROPERTY_ENABLE)
        break;
      // Change state to skip reading temperature if needed
      this->state_ = (needsTemperature) ? WAITING_FOR_TEMP_DATA : GOT_REQUESTED_DATA;
    case WAITING_FOR_TEMP_DATA:
      // Only read temperature if its needed
      if (this->state_ == WAITING_FOR_TEMP_DATA) {
        uint8_t tempDataReady;
        if(lis2dh12_temp_data_ready_get(&this->lis2dh12_dev_ctx_, &tempDataReady) != ST_SUCCESS) {
          ESP_LOGE(TAG, "Failed to read status register");
          this->state_ = ERROR_NOT_RESPONDING;
          this->mark_failed();
          return;
        }
        // If no data ready, return and repeat in next loop
        if (tempDataReady != PROPERTY_ENABLE)
          break;
      }
    case GOT_REQUESTED_DATA:
      // If got requested data, read sensor data and notify listeners
      // Executed only if entered directly thru this switch or from above case
      SensorData sensor_data = this->get_sensor_data(this->current_config_.temperature_en == LIS2DH12_TEMP_ENABLE);
      this->state_ = READY_OK;
      this->sensor_listeners_on_sensor_update(sensor_data);
      break;
    default:
      ESP_LOGE(TAG, "Unexpected state: %d", this->state_);
      this->state_ = ERROR;
      this->mark_failed();
      return;
  }
  
  this->status_clear_warning();
}

bool LIS2DH12Component::apply_device_configuration(LIS2DH12Config config, bool force) {
  if (this->state_ == CONFIGURING_DEVICE) {
    ESP_LOGE(TAG, "Invalid state, configuration already in progress.");
    this->state_ = ERROR_NOT_CONFIGURED;
    return false;
  }
  
  if( !force && (this->current_config_ == config) ) {
    ESP_LOGW(TAG, "Configuration already applied.");
    return true;
  }

  StateCode oldState = state_;
  this->state_ = CONFIGURING_DEVICE;

  ESP_LOGCONFIG(TAG, "Applying new configuration:");
  LIS2DH12Config oldConfig = this->current_config_;
  LIS2DH12Config newConfig = this->current_config_;

  LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.block_data_update, config.block_data_update, force,
                                    "  Block data update...", lis2dh12_block_data_update_set(&this->lis2dh12_dev_ctx_, config.block_data_update), 
                                    "Failed enabling block data update.", "  Block data update already set.");

  LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.operation_mode, config.operation_mode, force,
                                    "  Operating mode...", lis2dh12_operating_mode_set(&this->lis2dh12_dev_ctx_, config.operation_mode), 
                                    "Failed setting operating mode.", "  Operating mode already set.");

  if (this->state_ == CONFIGURING_DEVICE) 
    ESP_LOGCONFIG(TAG, "  Sensor:");

  LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.data_rate, config.data_rate, force,
                                    "    Data rate...", lis2dh12_data_rate_set(&this->lis2dh12_dev_ctx_, config.data_rate),
                                    "Failed setting data rate.", "    Data rate already set.");

  LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.full_scale_range, config.full_scale_range, force,
                                    "    Full scale range...", lis2dh12_full_scale_set(&this->lis2dh12_dev_ctx_, config.full_scale_range),
                                    "Failed setting full scale range.", "    Full scale range already set.");
  
  LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.temperature_en, config.temperature_en, force,
                                    "    Temperature...", lis2dh12_temperature_meas_set(&this->lis2dh12_dev_ctx_, config.temperature_en),
                                    "Failed setting temperature sensor.", "    Temperature sensor already set.");

  if ( (this->state_ == CONFIGURING_DEVICE) ||
       (newConfig.click != config.click) || 
        force ) {
    ESP_LOGCONFIG(TAG, "  Click:");

    LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.click.interrupt_mode, config.click.interrupt_mode, force,
                                    "    Interrupt mode...", lis2dh12_tap_notification_mode_set(&this->lis2dh12_dev_ctx_, config.click.interrupt_mode),
                                    "Failed setting interrupt mode.", "    Interrupt mode already set.");

    LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.click.threshold, config.click.threshold, force,
                                    "    Threshold...", lis2dh12_tap_threshold_set(&this->lis2dh12_dev_ctx_, config.click.threshold),
                                    "Failed setting threshold.", "    Threshold already set.");

    if ( (this->state_ == CONFIGURING_DEVICE) || 
         (newConfig.click.double_en != config.click.double_en) || 
         (newConfig.click.axis != config.click.axis) || 
          force ) {
      lis2dh12_click_cfg_t click_cfg; // interrupts, maybe not needed 
      bool doubleClickEnabled = (config.click.double_en == PROPERTY_ENABLE);

      if ( (this->state_ == CONFIGURING_DEVICE) || 
           (lis2dh12_tap_conf_get(&this->lis2dh12_dev_ctx_, &click_cfg) != ST_SUCCESS) ) {
        ESP_LOGE(TAG, "Failed getting axis and double click configuration.");
        this->state_ = ERROR_NOT_CONFIGURED;
      }

      if ( (this->state_ == CONFIGURING_DEVICE) ||
           (newConfig.click.axis != config.click.axis) || 
            force ) {
        ESP_LOGCONFIG(TAG, "    Axis:");

        if ( (newConfig.click.axis.x_en != config.click.axis.x_en) || force ) {
          ESP_LOGCONFIG(TAG, "      X axis...");
          click_cfg.xd = doubleClickEnabled ? config.click.axis.x_en : PROPERTY_DISABLE;
          click_cfg.xs = !doubleClickEnabled ? config.click.axis.x_en : PROPERTY_DISABLE;
        }

        if ( (newConfig.click.axis.y_en != config.click.axis.y_en) || force ) {
          ESP_LOGCONFIG(TAG, "      Y axis...");
          click_cfg.yd = doubleClickEnabled ? config.click.axis.y_en : PROPERTY_DISABLE;
          click_cfg.ys = !doubleClickEnabled ? config.click.axis.y_en : PROPERTY_DISABLE;
        }

        if ( (newConfig.click.axis.z_en != config.click.axis.z_en) || force ) {
          ESP_LOGCONFIG(TAG, "      Z axis...");
          click_cfg.zd = doubleClickEnabled ? config.click.axis.z_en : PROPERTY_DISABLE;
          click_cfg.zs = !doubleClickEnabled ? config.click.axis.z_en : PROPERTY_DISABLE;
        }
      }

      if ( (this->state_ == CONFIGURING_DEVICE) ||
           (newConfig.click.double_en != config.click.double_en) || 
            force ) {
        if(doubleClickEnabled)
          ESP_LOGCONFIG(TAG, "    Double click...");
        else
          ESP_LOGCONFIG(TAG, "    Single click...");
      }

      if ( (this->state_ == CONFIGURING_DEVICE) ||
           (lis2dh12_tap_conf_set(&this->lis2dh12_dev_ctx_, &click_cfg) != ST_SUCCESS) ) {
        ESP_LOGE(TAG, "Failed setting axis and click type configuration.");
        this->state_ = ERROR_NOT_CONFIGURED;
      }
    }

    if( (this->state_ == CONFIGURING_DEVICE) ||
        (newConfig.click.timing != config.click.timing) || 
         force ) {
      ESP_LOGCONFIG(TAG, "    Timing:");

      LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.click.timing.time_limit, config.click.timing.time_limit, force,
                                    "      Time limit...", lis2dh12_shock_dur_set(&this->lis2dh12_dev_ctx_, config.click.timing.time_limit),
                                    "Failed setting time limit.", "      Time limit already set.");

      LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.click.timing.time_window, config.click.timing.time_window, force,
                                    "      Time window...", lis2dh12_double_tap_timeout_set(&this->lis2dh12_dev_ctx_, config.click.timing.time_window),
                                    "Failed setting time window.", "      Time window already set.");

      LIS2DH12_CONFIGURATION_ERROR_CHECK(newConfig.click.timing.time_latency, config.click.timing.time_latency, force,
                                    "      Time latency...", lis2dh12_quiet_dur_set(&this->lis2dh12_dev_ctx_, config.click.timing.time_latency),
                                    "Failed setting time latency.", "      Time latency already set.");
    }
  }

  if(this->state_ == ERROR_NOT_CONFIGURED) {
    this->current_config_ = newConfig;
    ESP_LOGE(TAG, "Failed to set configuration.");
    return false;
  }
  
  this->set_timeout("configDone", TURN_ON_DELAY_MS, [this, newConfig, oldConfig, oldState]() { 
    uint8_t reference;
    if (lis2dh12_filter_reference_get(&lis2dh12_dev_ctx_, &reference) != ST_SUCCESS) {
      ESP_LOGE(TAG, "Failed reseting filter block");
      this->state_ = ERROR_NOT_CONFIGURED;
      this->mark_failed();
      return;
    }

    ESP_LOGD(TAG, "New configuration set.");
    this->current_config_ = newConfig;
    if (this->state_ == CONFIGURING_DEVICE) {
      this->state_ = oldState; 
    } else {
      ESP_LOGW(TAG, "State changed from %s to %s during configuration timeout", oldState, this->state_);
    }

    if (this->click_listeners_.size() > 0 && !this->current_config_.click.is_enabled()) {
      ESP_LOGW(TAG, "Click listeners present but click is disabled");
    }

    if (this->sensor_listeners_.size() > 0 && this->current_config_.data_rate == LIS2DH12_POWER_DOWN) {
      ESP_LOGW(TAG, "Sensor listeners present but device is powered down");
    }

    this->general_listeners_on_config_change(newConfig, oldConfig);
  });

  return true;
}

/*void LIS2DH12Component::update() {
  ESP_LOGV(TAG, "Updating LIS2DH12...");

  if(!setup_ready_ && this->status_has_warning()) {
    ESP_LOGW(TAG, "Waiting for startup to complete...");
    return;
  }

  float temperature = 0.0f;
  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;

  if (this->temperature_sensor_set_) {
    temperature = this->get_temperature_celsius();
  }

  ESP_LOGD(TAG,
           "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, temp=%.3f°C",
           accel_x, accel_y, accel_z, temperature);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);

  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);

  this->status_clear_warning();
}*/


float LIS2DH12Component::get_temperature_celsius()
{
  ESP_LOGV(TAG, "Reading Temperature...");

  lis2dh12_op_md_t opMode;
  // Read the operation mode
  if (lis2dh12_operating_mode_get(&this->lis2dh12_dev_ctx_, &opMode) != 0) {
    ESP_LOGE(TAG, "Operation mode read failed");
    this->state_ = ERROR_NOT_RESPONDING;
    this->mark_failed();
    return NAN;
  }  

  int16_t rawData;
  // Read the raw temperature data
  if (lis2dh12_temperature_raw_get(&this->lis2dh12_dev_ctx_, &rawData) != 0) {
    ESP_LOGE(TAG, "Temperature data read failed");
    this->state_ = ERROR_NOT_RESPONDING;
    this->mark_failed();
    return NAN;
  }

  float_t temperatureC;
  // Convert the raw temperature data to Celsius based on the operation mode
  switch (opMode)
  {
    case LIS2DH12_HR_12bit: // High resolution mode
      temperatureC = lis2dh12_from_lsb_hr_to_celsius(rawData);
      break;
    case LIS2DH12_NM_10bit: // Normal mode
      temperatureC = lis2dh12_from_lsb_nm_to_celsius(rawData);
      break;
    case LIS2DH12_LP_8bit: // Low power mode
      temperatureC = lis2dh12_from_lsb_lp_to_celsius(rawData);
      break;
    default:
      ESP_LOGW(TAG, "Invalid operation mode");
      return NAN;
  }

  // Return the temperature in Celsius
  return temperatureC;
}

bool LIS2DH12Component::is_device_present() {
  uint8_t device_id;

  // Read the device ID of the LIS2DH12 sensor
  if (lis2dh12_device_id_get(&lis2dh12_dev_ctx_, &device_id) != ST_SUCCESS) {
    ESP_LOGE(TAG, "Failed to read device ID");
    this->state_ = ERROR_NOT_RESPONDING;
    return false;
  }

  // Check if the device ID matches the expected value
  if (device_id != LIS2DH12_ID) {
    ESP_LOGE(TAG, "Device ID mismatch, got: 0x%02X, expected: 0x%02X", device_id, LIS2DH12_ID);
    this->state_ = ERROR_NOT_FOUND;
    return false;
  }

  // Log the device ID and expected value for comparison
  ESP_LOGV(TAG, "Device ID: 0x%02X, expected: 0x%02X", device_id, LIS2DH12_ID);
  return true;
}

size_t LIS2DH12Component::click_listeners_on_click(lis2dh12_click_src_t clickSource) {
  size_t count = this->click_listeners_.size();

  if (count == 0) {
    return count;
  }

  count = 0;

  for (auto *listener : this->click_listeners_) {
    if (listener != nullptr) {
      listener->on_click(clickSource);
      count++;
    } else { 
      ESP_LOGE(TAG, "Encountered a null listener.");
    }
  }

  ESP_LOGVV(TAG, "Called on_click on %zu listeners", count);

  return count;
}

size_t LIS2DH12Component::sensor_listeners_on_sensor_update(SensorData data) {
  size_t count = this->sensor_listeners_.size();

  if (count == 0) {
    return count;
  }

  count = 0;

  // Iterate over all registered sensor listeners
  for (auto *listener : this->sensor_listeners_) {
    if (listener != nullptr && listener->needs_update(nullptr)) {
      listener->on_sensor_update(data);
      count++;
    } else { 
      ESP_LOGE(TAG, "Encountered a null listener.");
    }
  }

  ESP_LOGVV(TAG, "Called on_sensor_update on %zu listeners", count);

  return count;
}

size_t LIS2DH12Component::general_listeners_on_config_change(LIS2DH12Config newConfig, LIS2DH12Config oldConfig, bool force) {
  // Check if there's a change in configuration or if notification is forced
  if ((oldConfig == newConfig) && !force) {
    ESP_LOGV(TAG, "No config change detected, skipping on_config_change");
    return 0;
  }

  size_t count = this->general_listeners_.size();

  if (count == 0) {
    ESP_LOGV(TAG, "No listeners found");
    return count;
  }

  count = 0;

  // Iterate over all registered listeners and notify them of the configuration change
  for (auto *listener : this->general_listeners_) {
    if (listener != nullptr) {
      listener->on_config_change(newConfig, oldConfig);
      count++;
    } else {
      ESP_LOGE(TAG, "Encountered a null listener.");
    }
  }

  ESP_LOGV(TAG, "Called on_config_change on %zu listeners", count);

  return count;
}

size_t LIS2DH12Component::general_listeners_on_ready()
{
  size_t count = this->general_listeners_.size();

  if (count == 0) {
    ESP_LOGV(TAG, "No listeners found");
    return count;
  }

  count = 0;

  // Iterate over all listeners and call their on_ready method
  for (auto *listener : this->general_listeners_) {
    if (listener != nullptr) {
      listener->on_ready();
      count++;
    } else {
      ESP_LOGE(TAG, "Encountered a null listener.");
    }
  }

  ESP_LOGV(TAG, "Called on_ready on %zu listeners", count);

  return count;
}

size_t LIS2DH12Component::general_listeners_on_failure()
{
  size_t count = this->general_listeners_.size();

  if (count == 0) {
    ESP_LOGV(TAG, "No listeners found");
    return count;
  }

  count = 0;

  // Iterate over all listeners and call their on_failure method
  for (auto *listener : this->general_listeners_) {
    if(listener != nullptr) {
      listener->on_failure();
      count++;
    } else {
      ESP_LOGE(TAG, "Encountered a null listener.");
    }
  }

  ESP_LOGV(TAG, "Called on_failure on %zu listeners", count);

  return count;
}

int32_t LIS2DH12Component::platform_write_reg(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  // Check if the handle or buffer pointer is null
  if (handle == nullptr || bufp == nullptr) {
    ESP_LOGE(TAG, "Invalid handle or buffer pointer");
    return i2c::ErrorCode::ERROR_INVALID_ARGUMENT;  
  }

  // Cast the handle to LIS2DH12Component type
  LIS2DH12Component *lis2dh12 = reinterpret_cast<LIS2DH12Component *>(handle);

  // Write the data to the specified register
  i2c::ErrorCode err = lis2dh12->write_register(reg, bufp, len);

  if (err != i2c::ErrorCode::ERROR_OK) {
    ESP_LOGE(TAG, "i2c error %d, while writing to register: 0x%02X, len: %d", err, reg, len);
  }

  return err;
}

int32_t LIS2DH12Component::platform_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  // Check if the handle or buffer pointer is null
  if (handle == nullptr || bufp == nullptr) {
    ESP_LOGE(TAG, "Invalid handle or buffer pointer");
    return i2c::ErrorCode::ERROR_INVALID_ARGUMENT;  
  }

  // For multi-byte reads, the first bit of the register address must be set to 1
  if (len > 1)
  {
    ESP_LOGVV(TAG, "Multi-byte read at reg: 0x%02X, len: %d", reg, len);
    reg |= 0x80;
  }

  // Cast the handle to LIS2DH12Component type
  LIS2DH12Component *lis2dh12 = reinterpret_cast<LIS2DH12Component *>(handle);

  // Read the data to the specified register
  i2c::ErrorCode err = lis2dh12->read_register(reg, bufp, len, false);

  if (err != i2c::ErrorCode::ERROR_OK) {
    ESP_LOGE(TAG, "i2c error %d, while reading register: 0x%02X, len: %d", err, reg, len);
  }

  return err;
}

}  // namespace lis2dh12
}  // namespace esphome
