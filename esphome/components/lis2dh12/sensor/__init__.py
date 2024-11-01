import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_TYPE,
    UNIT_METER_PER_SECOND_SQUARED,
    UNIT_CELSIUS,
    ICON_BRIEFCASE_DOWNLOAD,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_TEMPERATURE
)
from .. import lis2dh12_ns, LIS2DH12Component, CONF_LIS2DH12_ID

DEPENDENCIES = ["lis2dh12"]

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"

LIS2DH12AccelerationSensors = lis2dh12_ns.class_(
    "LIS2DH12AccelerationSensors", 
    cg.PollingComponent
)

LIS2DH12TemperatureSensor = lis2dh12_ns.class_(
    "LIS2DH12TemperatureSensor", 
    cg.PollingComponent,
    sensor.Sensor
)

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)

TYPE_ACCELEROMETER = "accelerometer"
TYPE_TEMPERATURE = "temperature"

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_ACCELEROMETER: cv.All(
                cv.Schema(
                {
                    cv.GenerateID(): cv.declare_id(LIS2DH12AccelerationSensors),
                    cv.GenerateID(CONF_LIS2DH12_ID): cv.use_id(LIS2DH12Component),
                    cv.Optional(CONF_ACCEL_X): accel_schema,
                    cv.Optional(CONF_ACCEL_Y): accel_schema,
                    cv.Optional(CONF_ACCEL_Z): accel_schema,
                    
                }
            )
            .extend(cv.polling_component_schema("60s")),
            cv.has_at_least_one_key(CONF_ACCEL_X, CONF_ACCEL_Y, CONF_ACCEL_Z)
        ),        
        TYPE_TEMPERATURE: sensor.sensor_schema(
            LIS2DH12TemperatureSensor,
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        )
        .extend(
            {
                cv.GenerateID(CONF_LIS2DH12_ID): cv.use_id(LIS2DH12Component),
            }
        )
        .extend(cv.polling_component_schema("60s")),
    },
    default_type=TYPE_ACCELEROMETER,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config) 
    lis2dh12 = await cg.get_variable(config[CONF_LIS2DH12_ID])
    
    if config[CONF_TYPE] == TYPE_ACCELEROMETER:
        for d in ["x", "y", "z"]:
            accel_key = f"accel_{d}"
            if accel_key in config:
                sens = await sensor.new_sensor(config[accel_key])
                cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))
    if config[CONF_TYPE] == TYPE_TEMPERATURE:
        await sensor.register_sensor(var, config)
        cg.add(lis2dh12.enable_internal_temperature_sensor())
        
    cg.add(lis2dh12.register_sensor_listener(var))
        

        
        
    
    
