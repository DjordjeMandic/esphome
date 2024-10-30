#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "lis2dh12.h"
#include "lis2dh12_reg.h"  // ST platform-independent driver

namespace esphome {
namespace lis2dh12 {

static const char *const TAG = "lis2dh12";

static const uint8_t TURN_ON_DELAY_MS = 7;

#define LIS2DH12_CONFIGURATION_ERROR_CHECK(conf_current, conf_new, force, conf_log, func, error, info) \
  if ( (conf_current != conf_new) || force) { \
    ESP_LOGCONFIG(TAG, conf_log); \
    if (func != ST_SUCCESS) { \
      ESP_LOGE(TAG, error); \
      this->state_ = ERROR_NOT_CONFIGURED; \
      this->mark_failed(); \
      return; \
    } \
    conf_current = conf_new; \
  } else { \
    ESP_LOGV(TAG, info); \
  }


void LIS2DH12Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LIS2DH12...");
  this->state_ = SEARCHING_FOR_DEVICE;
  this->internal_setup(0);
}


void LIS2DH12Component::internal_setup(uint8_t stage) {
  switch(stage) {
    case 0:
      // Identify the device
      ESP_LOGCONFIG(TAG, "  Identifying device...");
      this->state_ = SEARCHING_FOR_DEVICE;
      if (!this->is_device_present()) {
        ESP_LOGE(TAG, "Device not found");
        this->state_ = ERROR_NOT_FOUND;
        this->mark_failed();
        return;
      }
      this->set_timeout("configDev", 0, [this]() { this->internal_setup(1); });
      break;
    case 1:
      // Configure the device
      ESP_LOGCONFIG(TAG, "  Configuring device...");
      this->apply_device_configuration(this->new_config_, true);
      if (this->state_ == ERROR_NOT_CONFIGURED) {
        ESP_LOGE(TAG, "Failed configuring device");
        this->state_ = ERROR_NOT_CONFIGURED;
        this->mark_failed();
        return;
      }
      this->set_timeout("readyDev", TURN_ON_DELAY_MS, [this]() { this->internal_setup(2); });
    case 2:
      // The device is ready
      uint8_t reference;
      if (lis2dh12_filter_reference_get(&lis2dh12_dev_ctx, &reference) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed reseting filter block");
        this->state_ = ERROR_NOT_CONFIGURED;
        this->mark_failed();
        return;
      }
      this->state_ = READY_OK;
      ESP_LOGCONFIG(TAG, "  Setup complete.");
      break;
    default:
      ESP_LOGE(TAG, "Invalid internal setup stage");
      this->state_ = ERROR_NOT_CONFIGURED;
      this->mark_failed();
      return;
  }
}

void LIS2DH12Component::loop() {
  
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
    case StateCode::RESETTING_DEVICE:
      ESP_LOGCONFIG(TAG, "  State: Resetting device");
      break;
    case StateCode::CONFIGURING_DEVICE:
      ESP_LOGCONFIG(TAG, "  State: Configuring device");
      break;
    case StateCode::BUSY:
      ESP_LOGCONFIG(TAG, "  State: Busy");
      break;
    case StateCode::ERROR:
      ESP_LOGE(TAG, "  State: General error");
      break;
    case StateCode::ERROR_TIMEOUT:
      ESP_LOGE(TAG, "  State: Timeout exceeded");
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

  if(this->listeners_.size() > 0) {
    ESP_LOGCONFIG(TAG, "  Number of listeners: %u", this->listeners_.size());
  }

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Component failed!");
  }

//#ifdef USE_SENSOR
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
//#endif

  bool click_enabled = current_config_.click.is_enabled();
  ESP_LOGCONFIG(TAG, "  Click detection: %s", ONOFF(click_enabled));
  bool printClickConfig = click_enabled;
#ifdef USE_BINARY_SENSOR
  printClickConfig = true;
#endif
  if(printClickConfig) {
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

  /*LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);*/
}

// todo, add force set
void LIS2DH12Component::apply_device_configuration(LIS2DH12Config config, bool force) {
  if (this->state_ == CONFIGURING_DEVICE) {
    ESP_LOGE(TAG, "Invalid state, configuration already in progress.");
    this->state_ = ERROR_NOT_CONFIGURED;
    this->mark_failed();
    return;
  }
  
  if( !force && (this->current_config_ == config) ) {
    ESP_LOGW(TAG, "Configuration already applied.");
    return;
  }

  ESP_LOGCONFIG(TAG, "Applying new configuration:");
  StateCode oldState = state_;
  this->state_ = CONFIGURING_DEVICE;

  LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.block_data_update, config.block_data_update, force,
                                    "  Block data update...", lis2dh12_block_data_update_set(&this->lis2dh12_dev_ctx, config.block_data_update), 
                                    "Failed enabling block data update.", "  Block data update already set.");

  LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.operation_mode, config.operation_mode, force,
                                    "  Operating mode...", lis2dh12_operating_mode_set(&this->lis2dh12_dev_ctx, config.operation_mode), 
                                    "Failed setting operating mode.", "  Operating mode already set.");

  ESP_LOGCONFIG(TAG, "  Sensor:");

  LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.data_rate, config.data_rate, force,
                                    "    Data rate...", lis2dh12_data_rate_set(&this->lis2dh12_dev_ctx, config.data_rate),
                                    "Failed setting data rate.", "    Data rate already set.");

  LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.full_scale_range, config.full_scale_range, force,
                                    "    Full scale range...", lis2dh12_full_scale_set(&this->lis2dh12_dev_ctx, config.full_scale_range),
                                    "Failed setting full scale range.", "    Full scale range already set.");
  
  LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.temperature_en, config.temperature_en, force,
                                    "    Temperature...", lis2dh12_temperature_meas_set(&this->lis2dh12_dev_ctx, config.temperature_en),
                                    "Failed setting temperature sensor.", "    Temperature sensor already set.");

  if ( (this->current_config_.click != config.click) || force ) {
    ESP_LOGCONFIG(TAG, "  Click:");

    LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.click.interrupt_mode, config.click.interrupt_mode, force,
                                    "    Interrupt mode...", lis2dh12_tap_notification_mode_set(&this->lis2dh12_dev_ctx, config.click.interrupt_mode),
                                    "Failed setting interrupt mode.", "    Interrupt mode already set.");

    LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.click.threshold, config.click.threshold, force,
                                    "    Threshold...", lis2dh12_tap_threshold_set(&this->lis2dh12_dev_ctx, config.click.threshold),
                                    "Failed setting threshold.", "    Threshold already set.");

    if ( (this->current_config_.click.double_en != config.click.double_en) || 
         (this->current_config_.click.axis != config.click.axis) || 
         force ) {
      lis2dh12_click_cfg_t click_cfg; // interrupts, maybe not needed 
      bool doubleClickEnabled = (config.click.double_en == PROPERTY_ENABLE);

      if (lis2dh12_tap_conf_get(&this->lis2dh12_dev_ctx, &click_cfg) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed getting axis and double click configuration.");
        this->state_ = ERROR_NOT_CONFIGURED;
        this->mark_failed();
        return;
      }

      if ( (this->current_config_.click.axis != config.click.axis) || force ) {
        ESP_LOGCONFIG(TAG, "    Axis:");

        if ( (this->current_config_.click.axis.x_en != config.click.axis.x_en) || force ) {
          ESP_LOGCONFIG(TAG, "      X axis...");
          click_cfg.xd = doubleClickEnabled ? config.click.axis.x_en : PROPERTY_DISABLE;
          click_cfg.xs = !doubleClickEnabled ? config.click.axis.x_en : PROPERTY_DISABLE;
        }

        if ( (this->current_config_.click.axis.y_en != config.click.axis.y_en) || force ) {
          ESP_LOGCONFIG(TAG, "      Y axis...");
          click_cfg.yd = doubleClickEnabled ? config.click.axis.y_en : PROPERTY_DISABLE;
          click_cfg.ys = !doubleClickEnabled ? config.click.axis.y_en : PROPERTY_DISABLE;
        }

        if ( (this->current_config_.click.axis.z_en != config.click.axis.z_en) || force ) {
          ESP_LOGCONFIG(TAG, "      Z axis...");
          click_cfg.zd = doubleClickEnabled ? config.click.axis.z_en : PROPERTY_DISABLE;
          click_cfg.zs = !doubleClickEnabled ? config.click.axis.z_en : PROPERTY_DISABLE;
        }
      }

      if ( (this->current_config_.click.double_en != config.click.double_en) || force ) {
        if(doubleClickEnabled)
          ESP_LOGCONFIG(TAG, "    Double click...");
        else
          ESP_LOGCONFIG(TAG, "    Single click...");
      }

      if (lis2dh12_tap_conf_set(&this->lis2dh12_dev_ctx, &click_cfg) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed setting axis and click type configuration.");
        this->state_ = ERROR_NOT_CONFIGURED;
        this->mark_failed();
        return;
      }
    }

    if( (this->current_config_.click.timing != config.click.timing) || force ) {
      ESP_LOGCONFIG(TAG, "    Timing:");

      LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.click.timing.time_limit, config.click.timing.time_limit, force,
                                    "      Time limit...", lis2dh12_shock_dur_set(&this->lis2dh12_dev_ctx, config.click.timing.time_limit),
                                    "Failed setting time limit.", "      Time limit already set.");

      LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.click.timing.time_window, config.click.timing.time_window, force,
                                    "      Time window...", lis2dh12_double_tap_timeout_set(&this->lis2dh12_dev_ctx, config.click.timing.time_window),
                                    "Failed setting time window.", "      Time window already set.");

      LIS2DH12_CONFIGURATION_ERROR_CHECK(this->current_config_.click.timing.time_latency, config.click.timing.time_latency, force,
                                    "      Time latency...", lis2dh12_quiet_dur_set(&this->lis2dh12_dev_ctx, config.click.timing.time_latency),
                                    "Failed setting time latency.", "      Time latency already set.");
    }

    this->current_config_.click = config.click;
  }

  this->set_timeout("configDone", TURN_ON_DELAY_MS, [this, oldState, config]() { 
    this->current_config_ = config;
    this->state_ = oldState; 

    for (auto &listener : listeners_) {
      ESP_LOGV(TAG, "Notifying config change...");
      listener->on_config_change(this->current_config_);
    }
  });
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

/**
 * @brief Get the temperature in Celsius from the LIS2DH12 sensor.
 * 
 * This function reads the raw temperature data from the sensor, waits for new data to be available,
 * and then converts the raw data to Celsius based on the sensor's operating mode.
 * 
 * @return The temperature in Celsius if successful, NAN otherwise.
 */
/*float LIS2DH12Component::get_temperature_celsius()
{
  ESP_LOGV(TAG, "Reading Temperature...");

  // Check if the temperature sensor is set, if not, try enabling it
  if (!this->temperature_sensor_set_) {
    ESP_LOGW(TAG, "Temperature sensor not set");

    if (lis2dh12_temperature_meas_set(&lis2dh12_dev_ctx, LIS2DH12_TEMP_ENABLE) != 0) {
      ESP_LOGE(TAG, "Failed enabling temperature sensor");
      this->status_set_warning("TEMP_EN set fail");
      return NAN;
    }
  }

  // Wait for new temperature data to become available, with a timeout
  auto millisStart = millis();
  uint8_t newTempAvailable = 0;
  while ((newTempAvailable != 1) && ((millis() - millisStart) < LIS2DH12_TEMPERATURE_READ_TIMEOUT_MS)) {
    if (lis2dh12_temp_data_ready_get(&lis2dh12_dev_ctx, &newTempAvailable) != 0) {
      ESP_LOGE(TAG, "Temperature data ready read failed");
      this->status_set_warning("TDA get fail");
      return NAN;
    }

    yield();  // Allow other tasks to run
  }

  // Check if new temperature data is not available after timeout
  if (newTempAvailable == 0) {
    ESP_LOGE(TAG, "Timed out waiting for new temperature data");
    this->status_set_warning("TDA timeout");
    return NAN;
  }

  int16_t rawData;
  // Read the raw temperature data
  if (lis2dh12_temperature_raw_get(&lis2dh12_dev_ctx, &rawData) != 0) {
    ESP_LOGE(TAG, "Temperature raw data read failed");
    this->status_set_warning("Temp raw read fail");
    return NAN;
  }

  lis2dh12_op_md_t opMode;
  // Read the operation mode
  if (lis2dh12_operating_mode_get(&lis2dh12_dev_ctx, &opMode) != 0) {
    ESP_LOGE(TAG, "Operation mode read failed");
    this->status_set_warning("opMode read fail");
    return NAN;
  }

  float_t temperatureC;

  // Convert the raw temperature data to Celsius
  switch (opMode)
  {
    case LIS2DH12_HR_12bit: //High resolution mode
      temperatureC = lis2dh12_from_lsb_hr_to_celsius(rawData);
      break;
    case LIS2DH12_NM_10bit: //Normal mode
      temperatureC = lis2dh12_from_lsb_nm_to_celsius(rawData);
      break;
    case LIS2DH12_LP_8bit: //Low power mode
      temperatureC = lis2dh12_from_lsb_lp_to_celsius(rawData);
      break;
    default:
      ESP_LOGW(TAG, "Invalid operation mode");
      this->status_set_warning("opMode invalid");
      return NAN;
  }

  // Return the temperature in Celsius
  return temperatureC;
}*/

bool LIS2DH12Component::reset_device(uint8_t timeout_ms) {
  return true;
  // Trigger device reset
  if(lis2dh12_boot_set(&lis2dh12_dev_ctx, PROPERTY_ENABLE) != ST_SUCCESS) {
    ESP_LOGE(TAG, "Failed to trigger device reset");
    this->state_ = ERROR_NOT_RESPONDING;
    this->mark_failed();
    return false;
  }

  delay(TURN_ON_DELAY_MS);

  auto start_time = millis();
  uint8_t boot_state = 1;
  bool readAtLeastOnce = false;
  // Wait for the boot state to become 0 or until the timeout is reached
  while (!readAtLeastOnce || ((millis() - start_time < timeout_ms) && (boot_state != PROPERTY_DISABLE))) {
    if (lis2dh12_boot_get(&lis2dh12_dev_ctx, &boot_state) != ST_SUCCESS) {
      ESP_LOGE(TAG, "Failed to read boot state.");
      this->state_ = ERROR_NOT_RESPONDING;
      this->mark_failed();
      return false;
    }
    if(boot_state == PROPERTY_DISABLE) {
      ESP_LOGV(TAG, "Device reset successful");
      return true;
    }
    readAtLeastOnce = true;
    yield();  // Yield control to other tasks
  }

  // Check if the reset operation exceeded the timeout
  if(millis() - start_time > timeout_ms) {
    ESP_LOGE(TAG, "Reset timeout exceeded");
    this->state_ = ERROR_TIMEOUT;
    this->status_set_warning();
    return false;
  }

  return (boot_state == PROPERTY_DISABLE);  
}

bool LIS2DH12Component::is_device_present() {
  uint8_t device_id;

  // Read the device ID of the LIS2DH12 sensor
  if (lis2dh12_device_id_get(&lis2dh12_dev_ctx, &device_id) != ST_SUCCESS) {
    ESP_LOGE(TAG, "Failed to read device ID");
    this->state_ = ERROR_NOT_RESPONDING;
    this->mark_failed();
    return false;
  }

  // Check if the device ID matches the expected value
  if (device_id != LIS2DH12_ID) {
    ESP_LOGE(TAG, "Device ID mismatch, got: 0x%02X, expected: 0x%02X", device_id, LIS2DH12_ID);
    this->state_ = ERROR_NOT_FOUND;
    this->mark_failed();
    return false;
  }

  // Log the device ID and expected value for comparison
  ESP_LOGV(TAG, "Device ID: 0x%02X, expected: 0x%02X", device_id, LIS2DH12_ID);
  return true;
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
    ESP_LOGVV(TAG, "Multi-byte read at reg: 0x%02X", reg);
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
