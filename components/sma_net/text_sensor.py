import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import sma_net_ns, SmaNetComponent
from .const import CONF_CHANNEL, CONF_SMA_NET_ID

DEPENDENCIES = ["sma_net"]

SmaNetTextSensor = sma_net_ns.class_("SmaNetTextSensor", text_sensor.TextSensor)

CONFIG_SCHEMA = text_sensor.text_sensor_schema(SmaNetTextSensor).extend(
    {
        cv.GenerateID(CONF_SMA_NET_ID): cv.use_id(SmaNetComponent),
        cv.Required(CONF_CHANNEL): cv.string_strict,
    }
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    hub = await cg.get_variable(config[CONF_SMA_NET_ID])
    cg.add(hub.register_text_sensor(config[CONF_CHANNEL], var))
