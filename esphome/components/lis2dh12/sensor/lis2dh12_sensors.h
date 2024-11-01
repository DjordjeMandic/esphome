#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/helpers.h"
#include "../lis2dh12.h"

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

class LIS2DH12TemperatureSensor :   public PollingComponent,
                                    public sensor::Sensor,
                                    public LIS2DH12SensorListener {
    public:
        void call_setup() override;
        void dump_config() override;
        void update() override;
        float get_setup_priority() const override { return setup_priority::DATA; }

        void on_sensor_update(SensorData value) override;
        void on_ready() override;
        void on_failure() override;
        void on_config_change(LIS2DH12Config newConfig, LIS2DH12Config oldConfig) override;
    
    protected:
};

}  // namespace lis2dh12
}  // namespace esphome