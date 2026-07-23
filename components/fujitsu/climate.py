import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor, uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

from . import fujitsu_ns

CODEOWNERS = ["@jaimelaborda"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]

CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_HORIZONTAL_SWING = "horizontal_swing"
CONF_PRESETS = "presets"
CONF_LOG_RAW_FRAMES = "log_raw_frames"

FujitsuClimate = fujitsu_ns.class_(
    "FujitsuClimate", climate.Climate, uart.UARTDevice, cg.Component
)

CONFIG_SCHEMA = (
    climate.climate_schema(FujitsuClimate)
    .extend(
        {
            # Expose horizontal-swing options in Home Assistant. Leave false for
            # units without a horizontal louver (the firmware also auto-detects
            # this and ignores unsupported commands).
            cv.Optional(CONF_HORIZONTAL_SWING, default=False): cv.boolean,
            # Map the Powerful / Economy functions onto climate presets boost/eco.
            cv.Optional(CONF_PRESETS, default=True): cv.boolean,
            # Log every raw UART frame at DEBUG level (verbose; for bring-up only).
            cv.Optional(CONF_LOG_RAW_FRAMES, default=False): cv.boolean,
            # Optional outdoor-temperature sensor.
            cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await climate.register_climate(var, config)

    cg.add(var.set_horizontal_swing_supported(config[CONF_HORIZONTAL_SWING]))
    cg.add(var.set_presets_enabled(config[CONF_PRESETS]))
    cg.add(var.set_log_raw_frames(config[CONF_LOG_RAW_FRAMES]))

    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(var.set_outdoor_temperature_sensor(sens))
