#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "lis2dh12.h"
#include "lis2dh12_reg.h"  // ST platform-independent driver

namespace esphome {
namespace lis2dh12 {

static const char *const TAG = "lis2dh12";

void LIS2DH12Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LIS2DH12...");

  ESP_LOGCONFIG(TAG, "  Indentifying device...");
  this->state_ = SEARCHING_FOR_DEVICE;
  if (!is_device_present()) {
    ESP_LOGE(TAG, "Device not found");
    this->state_ = ERROR_NOT_FOUND;
    this->mark_failed();
    return;
  }

ESP_LOGCONFIG(TAG, "  Resetting device...");
  this->state_ = RESETTING_DEVICE;
  if (!reset_device()) {
    ESP_LOGE(TAG, "Failed resetting device");
    this->state_ = ERROR_NOT_RESPONDING;
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "  Waiting for startup, %d ms...", turnOnDelay_ms);
  this->status_set_warning("Startup in progress");
  this->set_timeout("turnOnDelay", turnOnDelay_ms, [this]() { 
    delay(turnOnDelay_ms); 
    this->setup_ready_ = true; 
    ESP_LOGI(TAG, "Startup complete");
    this->state_ = READY_OK;
    this->status_clear_warning();
  });
}

void LIS2DH12Compoennt::loop() {
  
}

void LIS2DH12Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LIS2DH12:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with LIS2DH12 failed!");
  }
  /*LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);*/
}

// todo
void LIS2DH12Component::apply_device_configuration(ConfigT config) {
  ESP_LOGCONFIG(TAG, "Applying new configuration:");
  StateCode oldState = state_;
  this->state_ = CONFIGURING_DEVICE;

  if (this->current_config_.block_data_update != config.block_data_update) {
    ESP_LOGCONFIG(TAG, "  Block data update..."); 
    if (lis2dh12_block_data_update_set(&this->lis2dh12_dev_ctx, config.block_data_update) != ST_SUCCESS) {
      ESP_LOGE(TAG, "Failed enabling block data update.");
      set_error_not_configured_and_mark_failed();
      return;
    }
  } else {
    ESP_LOGV(TAG, "  Block data update already set.");
  }

  if (this->current_config_.operation_mode != config.operation_mode) {
    ESP_LOGCONFIG(TAG, "  Operating mode...");
    if (lis2dh12_operating_mode_set(&this->lis2dh12_dev_ctx, config.operation_mode) != ST_SUCCESS) {
      ESP_LOGE(TAG, "Failed setting operating mode.");
      set_error_not_configured_and_mark_failed();
      return;
    }
  } else {
    ESP_LOGV(TAG, "  Operating mode already set.");
  }

  if (this->current_config_.sensor != config.sensor) {
    ESP_LOGCONFIG(TAG, "  Sensor:");

    if (this->current_config_.sensor.data_rate != config.sensor.data_rate) {
      ESP_LOGCONFIG(TAG, "    Data rate...");
      if (lis2dh12_data_rate_set(&this->lis2dh12_dev_ctx, config.sensor.data_rate) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed setting data rate.");
        set_error_not_configured_and_mark_failed();
        return;
      }
    } else {
      ESP_LOGV(TAG, "    Data rate already set.");
    }

    if (this->current_config_.sensor.temperature_en != config.sensor.temperature_en) {
      ESP_LOGCONFIG(TAG, "    Temperature...");
      if (lis2dh12_temperature_meas_set(&this->lis2dh12_dev_ctx, config.sensor.temperature_en) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed setting temperature sensor.");
        set_error_not_configured_and_mark_failed();
        return;
      }
    } else {
      ESP_LOGV(TAG, "    Temperature sensor already set.");
    }

    if (this->current_config_.sensor.accel != config.sensor.accel) {
      ESP_LOGCONFIG(TAG, "    Accelerometer:");

      if (this->current_config_.sensor.accel.x_en != config.sensor.accel.x_en) {
        ESP_LOGCONFIG(TAG, "      X axis...");
        if (lis2dh12_x_axis_set(&this->lis2dh12_dev_ctx, config.sensor.accel.x_en) != ST_SUCCESS) {
          ESP_LOGE(TAG, "Failed setting X axis.");
          set_error_not_configured_and_mark_failed();
          return;
        }
      } else {
        ESP_LOGV(TAG, "      X axis already set.");
      }

      if (this->current_config_.sensor.accel.y_en != config.sensor.accel.y_en) {
        ESP_LOGCONFIG(TAG, "      Y axis...");
        if (lis2dh12_y_axis_set(&this->lis2dh12_dev_ctx, config.sensor.accel.y_en) != ST_SUCCESS) {
          ESP_LOGE(TAG, "Failed setting Y axis.");
          set_error_not_configured_and_mark_failed();
          return;
        }
      } else {
        ESP_LOGV(TAG, "      Y axis already set.");
      }

      if (this->current_config_.sensor.accel.z_en != config.sensor.accel.z_en) {
        ESP_LOGCONFIG(TAG, "      Z axis...");
        if (lis2dh12_z_axis_set(&this->lis2dh12_dev_ctx, config.sensor.accel.z_en) != ST_SUCCESS) {
          ESP_LOGE(TAG, "Failed setting Z axis.");
          set_error_not_configured_and_mark_failed();
          return;
        }
      } else {
        ESP_LOGV(TAG, "      Z axis already set.");
      }

      if (this->current_config_.sensor.accel.full_scale_range != config.sensor.accel.full_scale_range) {
        ESP_LOGCONFIG(TAG, "      Full scale range...");
        if (lis2dh12_full_scale_set(&this->lis2dh12_dev_ctx, config.sensor.accel.full_scale_range) != ST_SUCCESS) {
          ESP_LOGE(TAG, "Failed setting full scale.");
          set_error_not_configured_and_mark_failed();
          return;
        }
      } else {
        ESP_LOGV(TAG, "      Full scale range already set.");
      }
    }

  }

  if (this->current_config_.click != config.click) {
    ESP_LOGCONFIG(TAG, "  Click:");

    if (this->current_config_.click.interrupt_mode != config.click.interrupt_mode) {
      ESP_LOGCONFIG(TAG, "    Interrupt mode...");
      if(lis2dh12_tap_notification_mode_set(&this->lis2dh12_dev_ctx, config.click.interrupt_mode) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed setting interrupt mode.");
        set_error_not_configured_and_mark_failed();
        return;
      }
    } else {
      ESP_LOGV(TAG, "    Interrupt mode already set.");
    }

    if (this->current_config_.click.threshold != config.click.threshold) {
      ESP_LOGCONFIG(TAG, "    Threshold...");
      if(lis2dh12_tap_threshold_set(&this->lis2dh12_dev_ctx, config.click.threshold) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed setting threshold.");
        set_error_not_configured_and_mark_failed();
        return;
      }
    } else {
      ESP_LOGV(TAG, "    Threshold already set.");
    }

    if (this->current_config_.click.double_en != config.click.double_en || this->current_config_.click.axis != config.click.axis) {
      lis2dh12_click_cfg_t click_cfg;

      if (lis2dh12_tap_conf_get(&this->lis2dh12_dev_ctx, &click_cfg) != ST_SUCCESS) {
        ESP_LOGE(TAG, "Failed getting axis and double click configuration.");
        set_error_not_configured_and_mark_failed();
        return;
      }

      if (this->current_config_.click.axis != config.click.axis) {
        ESP_LOGCONFIG(TAG, "    Axis:");

        if (this->current_config_.click.axis.x_en != config.click.axis.x_en) {
          ESP_LOGCONFIG(TAG, "      X axis...");
          click_cfg.xd = config.click.axis.x_en;
          click_cfg.xs = config.click.axis.x_en;
        }
      }

      bool doubleClickEnabled = false;
      if (this->current_config_.click.double_en != config.click.double_en) {
        ESP_LOGCONFIG(TAG, "    Double click...")
        doubleClickEnabled = (current_config_.click.double_en == PROPERTY_ENABLE);
      }

      if (doubleClickEnabled) {
        click_cfg.xs = PROPERTY_DISABLE;
        click_cfg.ys = PROPERTY_DISABLE;
        click_cfg.zs = PROPERTY_DISABLE;
      } else {
        click_cfg.xd = PROPERTY_DISABLE;
        click_cfg.yd = PROPERTY_DISABLE;
        click_cfg.zd = PROPERTY_DISABLE;
      }

    }
    



  }

  delay(turnOnDelay_ms); 

  uint8_t reference;
  if (lis2dh12_filter_reference_get(&lis2dh12_dev_ctx, &reference) != 0) {
    ESP_LOGE(TAG, "Failed reseting filter block");
    this->mark_failed();
    return;
  }
  
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
  // Trigger device reset
  if(lis2dh12_boot_set(&lis2dh12_dev_ctx, PROPERTY_ENABLE) != 0) {
    ESP_LOGE(TAG, "Failed to trigger device reset");
    return false;
  }

  delay(5); // 5ms boot delay

  auto start_time = millis();
  uint8_t boot_state = 1;
  bool readAtLeastOnce = false;
  // Wait for the boot state to become 0 or until the timeout is reached
  while (!readAtLeastOnce || ((millis() - start_time < timeout_ms) && (boot_state != PROPERTY_DISABLE))) {
    if (lis2dh12_boot_get(&lis2dh12_dev_ctx, &boot_state) != 0) {
      ESP_LOGE(TAG, "Failed to read boot state");
      return false;
    }
    readAtLeastOnce = true;
    yield();  // Yield control to other tasks
  }

  // Check if the reset operation exceeded the timeout
  if(millis() - start_time > timeout_ms) {
    ESP_LOGE(TAG, "Reset timeout exceeded");
  }

  return (boot_state == PROPERTY_DISABLE);  
}

bool LIS2DH12Component::is_device_present() {
  uint8_t device_id;

  // Read the device ID of the LIS2DH12 sensor
  if (lis2dh12_device_id_get(&lis2dh12_dev_ctx, &device_id) != 0) {
    ESP_LOGE(TAG, "Failed to read device ID");
    return false;
  }

  // Check if the device ID matches the expected value
  if (device_id != LIS2DH12_ID) {
    ESP_LOGE(TAG, "Device ID mismatch, got: 0x%02X, expected: 0x%02X", device_id, LIS2DH12_ID);
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
