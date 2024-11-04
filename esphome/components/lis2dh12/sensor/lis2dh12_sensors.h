#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/helpers.h"
#include "../lis2dh12.h"

// todo create base sensor listener class

namespace esphome {
namespace lis2dh12 {


// class LIS2DH12AccelerationSensors : public PollingComponent,
//                                     public LIS2DH12SensorListener {
//     public:
//         void setup() override;    
//         void dump_config() override;
//         void update() override;
//         float get_setup_priority() const override;

//         void on_sensor_update(SensorData value) override;
    
//         void set_accel_x_sensor(sensor::Sensor *sensor);
//         void set_accel_y_sensor(sensor::Sensor *sensor);    
//         void set_accel_z_sensor(sensor::Sensor *sensor);

// };

/**
 * @brief LIS2DH12TemperatureSensor class for managing the LIS2DH12 Temperature Sensor.
 * 
 * This class is responsible for interfacing with the LIS2DH12 Temperature Sensor. It handles
 * the setup, configuration logging, updates, and event handling for the sensor. It also manages
 * the polling mechanisms and responds to configuration changes and failures.
 */
class LIS2DH12TemperatureSensor : public PollingComponent,
                                  public sensor::Sensor,
                                  public LIS2DH12SensorListener {
    public:
        /**
         * @brief Constructor for LIS2DH12TemperatureSensor.
         * 
         * Initializes the LIS2DH12TemperatureSensor and marks it as needing temperature data.
         */
        LIS2DH12TemperatureSensor() {
            this->needs_temp_ = true;
        }

        /**
         * @brief Starts the temperature sensor.
         *
         * This method is called by the platform when the sensor is enabled. It calls
         * the user-provided setup() method and starts the poller to read the
         * temperature from the sensor.
         *
         * If the listener is disabled, the poller is not started and the component
         * status is set to warning.
         */
        void call_setup() override;

        /**
         * @brief Logs the configuration settings for the LIS2DH12 Temperature Sensor.
         *
         * This function outputs the current configuration of the LIS2DH12 Temperature Sensor,
         * including its listening status and update interval.
         */
        void dump_config() override;

        /**
         * @brief Updates the LIS2DH12 temperature sensor.
         *
         * This function checks if the sensor listener is active. If the listener
         * is off, it logs a warning, stops the poller, and sets the component status
         * to warning. Otherwise, it requests a sensor data update.
         */
        void update() override;

        float get_setup_priority() const override { return setup_priority::DATA; }
        
        /**
         * @brief Called when the sensor data has been updated.
         *
         * @param[in] value The updated sensor data.
         */
        void on_sensor_update(SensorData value) override;
        
        /**
         * @brief Called when the main component is ready.
         *
         * This function is triggered when the main component becomes ready. It clears
         * any existing warnings and initiates the polling process by starting the poller.
         */
        void on_ready() override;
        
        /**
         * @brief Called when the LIS2DH12 component has encountered a failure.
         *
         * This is called from the main LIS2DH12 component when it encounters an error.
         * It will also mark the sensor as failed.
         */
        void on_failure() override;

        /**
         * @brief Called when the LIS2DH12 component configuration changes.
         *
         * This function is called when the LIS2DH12 component configuration
         * changes. It is responsible for starting or stopping the poller if the
         * temperature sensor is enabled or disabled.
         *
         * @param newConfig The new configuration.
         * @param oldConfig The old configuration.
         */
        void on_config_change(LIS2DH12Config newConfig, LIS2DH12Config oldConfig) override;
};

}  // namespace lis2dh12
}  // namespace esphome