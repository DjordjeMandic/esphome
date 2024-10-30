#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/helpers.h"
#include "../lis2dh12.h"

namespace esphome {
namespace lis2dh12 {

class LIS2DH12AccelerationSensors : public PollingComponent,
                                    public LIS2DH12SensorListener {
    public:
        void setup() override;    
        void dump_config() override;
        void update() override;
        float get_setup_priority() const override;

        bool needs_update() override;
        void on_sensor_update(float value) override;
        
        void set_accel_x_sensor(sensor::Sensor *sensor);
        void set_accel_y_sensor(sensor::Sensor *sensor);    
        void set_accel_z_sensor(sensor::Sensor *sensor);

};

class LIS2DH12TemperatureSensor :   public PollingComponent,
                                    public sensor::Sensor,
                                    public LIS2DH12SensorListener {
    public:
        void setup() override;    
        void dump_config() override;
        void update() override;
        float get_setup_priority() const override;

        bool needs_update() override;
        void on_sensor_update(float value) override;
};

}  // namespace lis2dh12
}  // namespace esphome