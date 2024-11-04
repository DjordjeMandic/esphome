#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "../lis2dh12.h"
#include "lis2dh12_sensors.h"

namespace esphome {
namespace lis2dh12 {

static const char *const TAG = "lis2dh12.sensor.temp";

void LIS2DH12TemperatureSensor::call_setup() {
    this->setup();
    if (this->is_listening()) {
        this->status_clear_warning();
        this->start_poller();
    } else {
        this->status_set_warning("setup; listener off");
    }
}

void LIS2DH12TemperatureSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "LIS2DH12 Temperature Sensor:");
    ESP_LOGCONFIG(TAG, "  Listening: %s", TRUEFALSE(this->is_listening()));
    LOG_UPDATE_INTERVAL(this);
    LOG_SENSOR("  ", "Sensor:", this);
}

void LIS2DH12TemperatureSensor::update() {
    if (!this->is_listening()) {
        ESP_LOGW(TAG, "Listener is off, stopping poller...");
        this->status_set_warning("update; listener off");
        this->stop_poller();
        return;
    }
    
    ESP_LOGV(TAG, "Requesting update...");
    this->request_update();
}

void LIS2DH12TemperatureSensor::on_sensor_update(SensorData value) {
    if (this->is_failed() || !this->needs_update(nullptr))
        return;

    // Call the base class method to update the listener status
    LIS2DH12SensorListener::on_sensor_update(value);

    // Log the received value
    ESP_LOGD(TAG, "Got temperature: %.2f", value.temperature);

    // Publish the updated value
    this->publish_state(value.temperature);
}

void LIS2DH12TemperatureSensor::on_config_change(LIS2DH12Config newConfig, LIS2DH12Config oldConfig) {
    if (this->is_failed()) {
        return;
    }

    if (newConfig.temperature_en == oldConfig.temperature_en) {
        return;
    }

    if (newConfig.temperature_en != LIS2DH12_TEMP_ENABLE) {
        // If the temperature sensor is disabled, stop the poller and set an error status
        ESP_LOGE(TAG, "Temperature sensor disabled, stopping poller...");
        this->status_set_error("temp sensor off");
        this->stop_poller();
        return;
    }

    if (newConfig.temperature_en == LIS2DH12_TEMP_ENABLE) {
        // If the temperature sensor is enabled, start the poller if the listener is on
        this->status_clear_error();

        if (!this->is_listening())
        {
            ESP_LOGD(TAG, "Temperature sensor enabled, but not listening, not starting poller...");
            this->status_set_warning("tempSensOn; listener off");
            return;
        }

        if (!this->is_ready()) {
            ESP_LOGD(TAG, "Temperature sensor enabled but setup is not complete, not starting poller...");
            return;
        }

        ESP_LOGD(TAG, "Temperature sensor enabled, starting poller...");
        this->status_clear_warning();
        this->start_poller();
    }
}

void LIS2DH12TemperatureSensor::on_ready() {
    if (this->is_failed()) {
        return;
    }

    LIS2DH12SensorListener::on_ready();

    if (!this->is_ready()) {
        ESP_LOGD(TAG, "Listener ready but setup is not complete, not starting poller...");
        return;
    }

    ESP_LOGD(TAG, "Listener ready, starting poller...");
    this->status_clear_warning();
    this->start_poller();
}

void LIS2DH12TemperatureSensor::on_failure() {
    LIS2DH12SensorListener::on_failure();
    
    if (this->is_failed()) {
        return;
    }

    ESP_LOGE(TAG, "Listener encountered a failure");
    this->mark_failed();
}

}  // namespace lis2dh12
}  // namespace esphome