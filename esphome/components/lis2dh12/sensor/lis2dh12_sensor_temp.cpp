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
        this->start_poller();
    } else {
        ESP_LOGV(TAG, "Listener is off, keeping poller stopped...");
        this->status_set_warning("listener off");
    }
}

void LIS2DH12TemperatureSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "LIS2DH12 Temperature Sensor:");
    ESP_LOGCONFIG(TAG, "  Listening: %s", TRUEFALSE(this->is_listening()));
    LOG_UPDATE_INTERVAL(this);
    LOG_SENSOR("  ", "Sensor:", this);
}

void LIS2DH12TemperatureSensor::update() {
    if(this->is_listening()) {
        ESP_LOGV(TAG, "Requesting update...");
        this->request_update();
    } else {
        ESP_LOGW(TAG, "Listener is off, stopping poller...");
        this->status_set_warning("listener off");
        this->stop_poller();
    }
}

void LIS2DH12TemperatureSensor::on_sensor_update(SensorData value) {
    if (!this->needs_update())
        return;
    LIS2DH12SensorListener::on_sensor_update(value);
    ESP_LOGD(TAG, "Got temperature: %.2f", value.temperature);
    this->publish_state(value.temperature);
}

void LIS2DH12TemperatureSensor::on_config_change(LIS2DH12Config newConfig, LIS2DH12Config oldConfig) {
    if (newConfig.temperature_en != oldConfig.temperature_en) {
        if (newConfig.temperature_en != LIS2DH12_TEMP_ENABLE) {
            ESP_LOGE(TAG, "Temperature sensor disabled, stopping poller...");
            this->status_set_error("sensor disabled");
            this->stop_poller();
        } else {
            this->status_clear_error();
            if (this->is_listening()) {
                ESP_LOGD(TAG, "Temperature sensor enabled, starting poller...");
                this->start_poller();
            } else {
                ESP_LOGD(TAG, "Temperature sensor enabled, but not listening, not starting poller...");
            }
        }
    }
}

void LIS2DH12TemperatureSensor::on_ready() {
    LIS2DH12SensorListener::on_ready();
    ESP_LOGD(TAG, "Listener ready, starting poller...");
    this->status_clear_warning();
    this->start_poller();
}

void LIS2DH12TemperatureSensor::on_failure() {
    LIS2DH12SensorListener::on_failure();
    ESP_LOGE(TAG, "Listener encountered a failure; marking sensor as failed.");
    this->mark_failed();
}

}  // namespace lis2dh12
}  // namespace esphome