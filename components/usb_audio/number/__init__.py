import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_RESTORE_VALUE,
    CONF_STEP,
    ENTITY_CATEGORY_CONFIG,
    UNIT_DECIBEL,
)
from esphome.types import ConfigType

from .. import (
    CONF_USB_AUDIO_ID,
    USBAudioComponent,
    final_validate_one_per_hub,
    usb_audio_ns,
)

DEPENDENCIES = ["usb_audio"]

USBAudioGainNumber = usb_audio_ns.class_(
    "USBAudioGainNumber", number.Number, cg.Component
)

# UAC 1.0 volume is a signed 16-bit value in 1/256 dB.
GAIN_LIMIT_DB = 32767 / 256


def _validate_range(config: ConfigType) -> ConfigType:
    if config[CONF_MIN_VALUE] >= config[CONF_MAX_VALUE]:
        raise cv.Invalid("min_value must be below max_value")
    return config


CONFIG_SCHEMA = cv.All(
    number.number_schema(
        USBAudioGainNumber,
        unit_of_measurement=UNIT_DECIBEL,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:knob",
    )
    .extend(
        {
            cv.GenerateID(CONF_USB_AUDIO_ID): cv.use_id(USBAudioComponent),
            # The range and step until the device reports its own when it connects.
            cv.Optional(CONF_MIN_VALUE, default=-30.0): cv.float_range(
                min=-GAIN_LIMIT_DB, max=GAIN_LIMIT_DB
            ),
            cv.Optional(CONF_MAX_VALUE, default=30.0): cv.float_range(
                min=-GAIN_LIMIT_DB, max=GAIN_LIMIT_DB
            ),
            cv.Optional(CONF_STEP, default=0.5): cv.positive_float,
            cv.Optional(CONF_RESTORE_VALUE, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _validate_range,
)

FINAL_VALIDATE_SCHEMA = final_validate_one_per_hub("number")


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_USB_AUDIO_ID])
    cg.add(hub.enable_microphone_gain_control())
    var = await number.new_number(
        config,
        min_value=config[CONF_MIN_VALUE],
        max_value=config[CONF_MAX_VALUE],
        step=config[CONF_STEP],
    )
    await cg.register_component(var, config)
    await cg.register_parented(var, hub)
    if config[CONF_RESTORE_VALUE]:
        cg.add(var.set_restore_value(True))
