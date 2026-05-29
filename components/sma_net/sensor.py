import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from . import sma_net_ns, SmaNetComponent
from .const import CONF_CHANNEL, CONF_SMA_NET_ID

DEPENDENCIES = ["sma_net"]

CONF_INTERVAL = "interval"

SmaNetSensor = sma_net_ns.class_("SmaNetSensor", sensor.Sensor)

CONFIG_SCHEMA = sensor.sensor_schema(SmaNetSensor).extend(
    {
        cv.GenerateID(CONF_SMA_NET_ID): cv.use_id(SmaNetComponent),
        cv.Required(CONF_CHANNEL): cv.string_strict,
        cv.Optional(CONF_INTERVAL, default="slow"): cv.one_of("slow", "fast", lower=True),
    }
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    hub = await cg.get_variable(config[CONF_SMA_NET_ID])
    cg.add(hub.register_sensor(config[CONF_CHANNEL], var, config[CONF_INTERVAL] == "fast"))
