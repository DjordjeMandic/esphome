import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_THRESHOLD,
    CONF_TIMING
)
from .. import lis2dh12_ns, LIS2DH12Component, CONF_LIS2DH12_ID

CONF_TIME_LIMIT = "time_limit"
CONF_TIME_LATENCY = "time_latency"
CONF_TIME_WINDOW = "time_window"
CONF_DOUBLE_CLICK = "double_click"
CONF_AXIS = "axis"
CONF_AXIS_X = "x"
CONF_AXIS_Y = "y"
CONF_AXIS_Z = "z"
CONF_NEGATIVE_ONLY = "negative_only"

DEPENDENCIES = ["lis2dh12"]
AUTO_LOAD = ["binary_sensor"]

LIS2DH12Click = lis2dh12_ns.class_("LIS2DH12Click", binary_sensor.BinarySensor)

# todo
def validate_axis(config):
    if not any(value for value in config.values() if value != CONF_NEGATIVE_ONLY):
        raise cv.Invalid("At least one axis must be enabled")
    return config

# Define AXIS configuration schema
AXIS_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_AXIS_X, default=False): cv.boolean,
        cv.Optional(CONF_AXIS_Y, default=False): cv.boolean,
        cv.Optional(CONF_AXIS_Z, default=False): cv.boolean,
        cv.Optional(CONF_NEGATIVE_ONLY, default=False): cv.boolean,
    }
).validate(validate_axis)

TOLERANCE_MODE = {
    TYPE_PERCENTAGE: ToleranceMode.TOLERANCE_MODE_PERCENTAGE,
    TYPE_TIME: ToleranceMode.TOLERANCE_MODE_TIME,
}

TOLERANCE_SCHEMA = cv.typed_schema(
    {
        TYPE_PERCENTAGE: cv.Schema(
            {cv.Required(CONF_VALUE): cv.All(cv.percentage_int, cv.uint32_t)}
        ),
        TYPE_TIME: cv.Schema(
            {
                cv.Required(CONF_VALUE): cv.All(
                    cv.positive_time_period_microseconds,
                    cv.Range(max=TimePeriod(microseconds=4294967295)),
                )
            }
        ),
    },
    lower=True,
    enum=TOLERANCE_MODE,
)


# Custom validation function for timing
def validate_timing(config):
    # Check if double click is enabled
    if config.get(CONF_DOUBLE_CLICK, False):
        # Ensure both TIME_WINDOW and TIME_LATENCY are specified in CONF_TIMING
        timing_config = config.get(CONF_TIMING, {})
        if (CONF_TIME_WINDOW not in timing_config) or (CONF_TIME_LATENCY not in timing_config):
            raise cv.MultipleInvalid("Please specify both time latency and window for double click detection!")
    return config


# Define timing configuration schema
TIMING_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_TIME_LIMIT): cv.int_range(min=0, max=255),
        cv.Optional(CONF_TIME_LATENCY): cv.int_range(min=0, max=255),
        cv.Optional(CONF_TIME_WINDOW): cv.int_range(min=0, max=255),
    }
)

# Define the main configuration schema
CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(LIS2DH12Click).extend(
    cv.Schema(
        {
            cv.GenerateID(CONF_LIS2DH12_ID): cv.use_id(LIS2DH12Component),
            cv.Required(CONF_THRESHOLD): cv.int_range(min=0, max=127),
            cv.Required(CONF_AXIS): AXIS_CONFIG_SCHEMA,
            cv.Required(CONF_TIMING): TIMING_CONFIG_SCHEMA,
            cv.Optional(CONF_DOUBLE_CLICK, default=False): cv.boolean,
        }
    )
    .validate(validate_axis)
    .validate(validate_timing)
)

async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    hub = await cg.get_variable(config[CONF_LIS2DH12_ID])
    
    cg.add(var.set_threshold(config[CONF_THRESHOLD]))

    axis_config = config[CONF_AXIS]
    for d in ["x", "y", "z"]:
        axis_key = f"axis_{d}"
        if axis_key in axis_config:
            cg.add(getattr(var, f"set_axis_{d}")(axis_config[axis_key]))
            
    cg.add(var.set_negative_only(axis_config[CONF_NEGATIVE_ONLY]))

    timing_config = config[CONF_TIMING]
    cg.add(var.set_time_limit(timing_config[CONF_TIME_LIMIT]))
    
    cg.add(var.set_double_click(config[CONF_DOUBLE_CLICK]))
    if config[CONF_DOUBLE_CLICK]:
        cg.add(var.set_time_latency(timing_config.get(CONF_TIME_LATENCY)))
        cg.add(var.set_time_window(timing_config.get(CONF_TIME_WINDOW)))

    cg.add(hub.register_click_binary_sensor(var))
