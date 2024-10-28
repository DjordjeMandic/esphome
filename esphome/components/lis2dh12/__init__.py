import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_RANGE,
    CONF_DATA_RATE
)

CONF_LOW_POWER_MODE = "low_power_mode"

# 4D 6D - sensor
# click - binary sensor // 400Hz or higher

CODEOWNERS = ["@DjordjeMandic"]

DEPENDENCIES = ["i2c"]

MULTI_CONF = True

lis2dh12_ns = cg.esphome_ns.namespace("lis2dh12")
LIS2DH12Component = lis2dh12_ns.class_(
    "LIS2DH12Component", cg.Component, i2c.I2CDevice
)

CONF_LIS2DH12_ID = "lis2dh12_id"

DataRate = lis2dh12_ns.enum("DataRate")
Rate = {
    "1HZ": DataRate.ODR_1HZ,
    "10HZ": DataRate.ODR_10HZ,
    "25HZ": DataRate.ODR_25HZ,
    "50HZ": DataRate.ODR_50HZ,
    "100HZ": DataRate.ODR_100HZ,
    "200HZ": DataRate.ODR_200HZ,
    "400HZ": DataRate.ODR_400HZ,
    "1344HZ": DataRate.ODR_5kHz376_LP_1kHz344_NM_HP,
    "1620HZ": DataRate.ODR_1KHZ620_LP,
    "5376HZ": DataRate.ODR_5KHZ376_LP_1KHZ344_NM_HP
}

FullScaleRange = lis2dh12_ns.enum("FullScareRange")
Range = {
    "2G": FullScaleRange.FS_2G,
    "4G": FullScaleRange.FS_4G,
    "8G": FullScaleRange.FS_8G,
    "16G": FullScaleRange.FS_16G,
}

def validate_rate(data):
    """
    Validate the data rate configuration for the LIS2DH12 component.

    Args:
    - data: Dictionary containing the configuration data.

    Returns:
    - Validated configuration data.
    """
    # Extract configuration parameters
    low_power_mode = data.get(CONF_LOW_POWER_MODE)
    data_rate = data.get(CONF_DATA_RATE)

    # Validate the data rate based on power_save_mode
    if data_rate in ["1620HZ", "5376HZ"] and not low_power_mode:
        raise cv.Invalid("1620HZ and 5376HZ data rates can only be used with low_power_mode=True.")
    elif data_rate == "1344HZ" and low_power_mode:
        raise cv.Invalid("1344HZ data rate can only be used with low_power_mode=False.")

    return data

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LIS2DH12Component),
            cv.Optional(CONF_LOW_POWER_MODE, default=False): cv.boolean,
            cv.Optional(CONF_RANGE, default="2G"): cv.enum(Range, upper=True),
            cv.Optional(CONF_DATA_RATE, default="400HZ"): cv.enum(Rate, upper=True),
        }
    )
    .validate(validate_rate)
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x19))
)

FINAL_VALIDATE_SCHEMA = i2c.final_validate_device_schema(
    "lis2dh12",
    max_frequency = "400kHz"
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
        
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
